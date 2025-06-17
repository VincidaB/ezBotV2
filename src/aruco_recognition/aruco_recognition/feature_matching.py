import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_parameters = cv2.aruco.DetectorParameters_create()

# Modify the parameters here
aruco_parameters.minMarkerPerimeterRate = 0.01
aruco_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.33

# cf https://github.com/zsiki/Find-GCP for parameter tuning


class featureMatching(Node):

    def __init__(self):
        super().__init__('featureMatching')

        self.subscription3 = self.create_subscription(
            Image,
            '/robot1/camera3/img_raw',
            # '/robot1/camera1/img_raw',
            self.listener_callback,
            10)
        self.subscription3_mask = self.create_subscription(
            Image,
            '/robot1/camera3_segm/labels_map',
            # '/robot1/camera3_segm/colored_map',
            self.mask_callback,
            10)

        self.bridge = CvBridge()
        # storage of camera calibration data
        # width: 1280
        # height: 720
        # 0 distortion

        self.mtx = np.array(
            [[369.50206756591797, 0, 640],
             [0, 369.50210094451904, 360],
             [0, 0, 1]])
        self.dist = np.array([0, 0, 0, 0])
        self.mask_image = None
        self.masked_image = None
        self.cv_image = None
        self.empty_image = np.zeros((720, 1280, 3), np.uint8)

    def display_images(self):
        if self.mask_image is not None and self.cv_image is not None:

            mask = cv2.inRange(self.mask_image, (9, 0, 0), (9, 255, 255))
            masked_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
            top = cv2.hconcat([self.cv_image, masked_image])
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            bottom = cv2.hconcat([mask , self.mask_image])
            allofit = cv2.vconcat([top, bottom])
            cv2.namedWindow('all', cv2.WINDOW_NORMAL)
            cv2.imshow('all', allofit)


    def listener_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        cv2.waitKey(3)
        
    def mask_callback(self, msg):
        print('segm_image time stamp : ', time.time())
        self.mask_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # it is import to use the mask image that has the closest time stamp to the image
        # currently not automated but the images must come in the order of the cameras 
        # definition in the xacro file
        self.display_images()



def main(args=None):
    
    rclpy.init(args=args)

    feature_matching = featureMatching()

    rclpy.spin(feature_matching)

    # Destroy the node explicitly
    feature_matching.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()