import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_parameters = cv2.aruco.DetectorParameters_create()

# Modify the parameters here
aruco_parameters.minMarkerPerimeterRate = 0.01
aruco_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.33



# cf https://github.com/zsiki/Find-GCP for parameter tuning

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        
        self.subscription1 = self.create_subscription(
            Image,
            '/robot1/camera1',
            self.listener_callback,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            '/robot1/camera2',
            self.listener_callback1,
            10)
        self.subscription3 = self.create_subscription(
            Image,
            '/robot1/camera3',
            self.listener_callback2,
            10)
        
        self.bridge = CvBridge()
        self.images = [None, None, None]
        self.recording = True
        #Image shape: (540, 2880, 3)
        self.out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 30, (2880, 540))

    def show_images(self):
        if all(image is not None for image in self.images):
            concatenated_image = cv2.hconcat(self.images)
            concatenated_image = cv2.resize(concatenated_image, (0, 0), fx=0.75, fy=0.75)
            print('Image shape:', concatenated_image.shape)
            cv2.imshow('Cameras', concatenated_image)
            cv2.waitKey(3)
            if self.recording:
                self.out.write(concatenated_image)



    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = arucoDetection(cv_image)
        self.images[0] = cv_image
        self.show_images()

    def listener_callback1(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = arucoDetection(cv_image)
        self.images[1] = cv_image
        self.show_images()

    def listener_callback2(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = arucoDetection(cv_image)
        self.images[2] = cv_image
        self.show_images()




def arucoDetection(frame):


    (corners, ids, rejected) = cv2.aruco.detectMarkers(
      frame, aruco_dictionary, parameters=aruco_parameters)

    if len(corners) > 0:

        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(frame, str(markerID),
              (topLeft[0], topLeft[1] - 15),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.9, (0, 255, 255), 2)
              

            print("[INFO] ArUco marker ID: {}".format(markerID))
    
   # Display the rejected ones:
    cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(100, 0, 240))

    
    return frame



def main(args=None):
    
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()





if __name__ == '__main__':
    main()