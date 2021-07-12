import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import argparse
import cv2
import sys
import imutils

class ArucoPublisher(Node):

    """
    Aruco image publisher test fixture.
    Publishes test data on topics expected by detect_aruco_video nodes:
       - image_raw
    """
    
    def __init__(self,image_path):
        super().__init__('aruco_publisher')
        self.publisher = self.create_publisher(Image,'image_raw',10)
        timer_period = 0.5 # seconds
        self.bridge = CvBridge()
        self.image = cv2.imread(image_path)
        self.image = imutils.resize(self.image, width=600)
        self.image = cv2.copyMakeBorder(self.image,20,20,20,20,cv2.BORDER_CONSTANT,value=[255,255,255])

        self.timer = self.create_timer(timer_period,self.timer_callback)
        

    def timer_callback(self):
        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    
if __name__ == '__main__':

    rclpy.init()
    image_path = sys.argv[1]
    ar_publisher = ArucoPublisher(image_path)

    try:
        rclpy.spin(ar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ar_publisher.destroy_node()
        rclpy.shutdown()
