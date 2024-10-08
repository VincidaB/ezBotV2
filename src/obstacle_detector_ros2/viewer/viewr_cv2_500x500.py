from obstacle_msgs.msg import Obstacles

import cv2
import rclpy
from rclpy.node import Node
import numpy as np

class Viewer:
    def __init__(self):
        self.node = rclpy.create_node('viewer')
        self.node.create_subscription(Obstacles, '/raw_obstacles', self.callback, 10)
        self.node.get_logger().info('viewer is ready')

        # create window
        self.size_x = 1000
        cv2.namedWindow('viewer', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('viewer', self.size_x , self.size_x)
        self.window = np.zeros((self.size_x, self.size_x, 3), np.uint8)

    def callback(self, msg: Obstacles):
        lines = []
        for line in msg.segments:
            x1 = line.first_point.x
            y1 = line.first_point.y
            x2 = line.last_point.x
            y2 = line.last_point.y
            lines.append([x1, y1, x2, y2])

        circles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            r = circle.radius
            circles.append([x, y, r])


        print('lines: ', len(lines))
        print('circles: ', len(circles))

        self.draw_lines(lines, circles)

    def draw_lines(self, lines, circles):
        mag = 100000000
        circle_mag = 2
        for line in lines:
            x1 = int(line[0] * mag) + self.size_x // 2
            y1 = int(line[1] * mag) + self.size_x // 2
            x2 = int(line[2] * mag) + self.size_x // 2
            y2 = int(line[3] * mag) + self.size_x // 2
            cv2.line(self.window, (x1, y1), (x2, y2), (255, 0, 0), 5)

        for circle in circles:
            x = int(circle[0] * mag) + self.size_x // 2
            y = int(circle[1] * mag) + self.size_x // 2
            r = int(circle[2] * circle_mag)
            cv2.circle(self.window, (x, y), r, (255, 0, 0), -1)

        # ウィンドウを更新
        cv2.imshow('viewer', self.window)
        cv2.waitKey(1)


if __name__ == '__main__':
    rclpy.init()
    viewer = Viewer()
    rclpy.spin(viewer.node)
    rclpy.shutdown()