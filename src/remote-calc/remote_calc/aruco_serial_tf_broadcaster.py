#!/usr/bin/env python3
import threading
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np
import cv2
import serial


class ArucoSerialTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_serial_tf_broadcaster')

        # --- parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 1152000)
        # camera_matrix as flat list of 9 floats [fx, 0, cx, 0, fy, cy, 0,0,1]
        
        #TODO this needs fixing, I just copied the values from x to y to make everything work, and this needs an explaination
        #! TODO is this even distorted coordinates that we receive from the jevois ?!
        self.declare_parameter('camera_matrix',
            [ 405.08030285784002, 0., 405.08030285784002, 0., 539.10689419700418, 539.10689419700418,
       0., 0., 1. ]
             )
        # assume zero distortion by default
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])

        # per‐tag side lengths (m), keyed by ID; plus a default
        self.declare_parameter('default_tag_size', 0.10)

        # camera optical frame
        self.declare_parameter('camera_frame', 'camera1_link_optical')
        # prefix for published child frames
        self.declare_parameter('tag_frame_prefix', 'aruco_')

        # fetch params
        port   = self.get_parameter('serial_port').value
        baud   = self.get_parameter('baud_rate').value
        cm_list = self.get_parameter('camera_matrix').value
        dc_list = self.get_parameter('dist_coeffs').value

        self.tag_sizes = {1: 0.07, 6: 0.07, 20: 0.1, 21: 0.1, 22: 0.1, 23: 0.1}
        self.default_tag_size = float(self.get_parameter('default_tag_size').value)
    
        # keep track of which unknown tags we've warned about
        self._warned_unknown_tags = set()

        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame_prefix = self.get_parameter('tag_frame_prefix').value

        # build numpy intrinsics
        self.camera_matrix = np.array(cm_list, dtype=float).reshape(3,3)
        self.dist_coeffs   = np.array(dc_list, dtype=float).reshape(-1,1)

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # try to open serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Opened serial on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")
            sys.exit(1)

        # start reader thread
        self._lock = threading.Lock()
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    def _read_loop(self):
        frame_time = None
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                continue

            if not line:
                continue

            if line == 'MARK START':
                # stamp all following tag TFs with this ROS time
                frame_time = self.get_clock().now().to_msg()
                continue

            if line == 'MARK STOP':
                frame_time = None
                continue

            # parse data line
            # e.g.: "D2 U21 4 -665.8 248.2 -552.2 246.8 -551.1 336.2 -664.7 337.6"
            toks = line.split()
            if len(toks) < 11 or not toks[1].startswith('U'):
                continue

            try:
                tag_id = int(toks[1][1:])
                n_corners = int(toks[2])
                pts = [float(x) for x in toks[3:3+2*n_corners]]
                corners = np.array(pts, dtype=float).reshape(n_corners, 2)
            except Exception as e:
                self.get_logger().warn(f"Failed parsing line '{line}': {e}")
                continue

            if frame_time is None:
                # haven't seen MARK START yet
                continue

            # convert standardized coords [-1000,1000] to pixels
            # assume standardized x,y where (0,0) maps to principal point
            # and ±1000 maps to ±cx or ±cy
            cx = self.camera_matrix[0,2]
            cy = self.camera_matrix[1,2]
            sx = corners[:,0]
            sy = corners[:,1]
            px = (sx / 1000.0) * cx + cx
            py = - (sy / 1000.0) * cy + cy
            img_pts = np.vstack([px, py]).T.astype(np.float64)

            # object points of a square tag in its own plane, CCW from top-left:
            # pick the size for this tag (or default), warn once if unknown
            s = float(self.tag_sizes.get(tag_id, self.default_tag_size))
            if tag_id not in self.tag_sizes and tag_id not in self._warned_unknown_tags:
                self.get_logger().warning(
                    f"Tag ID {tag_id} not in tag_sizes; using default {self.default_tag_size:.3f} m"
                )
                self._warned_unknown_tags.add(tag_id)

            obj_pts = np.array([
                [-s/2,  s/2, 0],
                [ s/2,  s/2, 0],
                [ s/2, -s/2, 0],
                [-s/2, -s/2, 0]
            ], dtype=np.float64)

            # solvePnP
            success, rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success:
                self.get_logger().warn(f"PnP failed for tag {tag_id}")
                continue

            # rotation vector → rotation matrix → quaternion
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4, dtype=np.float64)
            T[0:3,0:3] = R
            q = cv2.RQDecomp3x3(R)[0]  # alternative, but let's use quaternion from matrix
            # better: use tf_transformations if available
            try:
                from tf_transformations import quaternion_from_matrix
                quat = quaternion_from_matrix(T)
            except ImportError:
                # fallback: normalize rvec to quaternion
                angle = np.linalg.norm(rvec)
                axis  = (rvec / angle).flatten() if angle != 0 else np.array([0,0,1])
                qw = np.cos(angle/2)
                qxyz = axis * np.sin(angle/2)
                quat = np.array([qxyz[0], qxyz[1], qxyz[2], qw])

            # build and send TransformStamped
            tmsg = TransformStamped()
            tmsg.header.stamp = frame_time
            tmsg.header.frame_id = self.camera_frame
            tmsg.child_frame_id  = f"{self.tag_frame_prefix}{tag_id}"
            tmsg.transform.translation.x = float(tvec[0])
            tmsg.transform.translation.y = float(tvec[1])
            tmsg.transform.translation.z = float(tvec[2])
            tmsg.transform.rotation.x = float(quat[0])
            tmsg.transform.rotation.y = float(quat[1])
            tmsg.transform.rotation.z = float(quat[2])
            tmsg.transform.rotation.w = float(quat[3])

            self.br.sendTransform(tmsg)

    def destroy_node(self):
        # clean up serial
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSerialTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
