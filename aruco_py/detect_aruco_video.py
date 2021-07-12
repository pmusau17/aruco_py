import argparse
import cv2
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# import list of aruco tags from OpenCV

from .markers.tag_definitions import ARUCO_DICT


class DetectAruco(Node):

    def __init__(self, args):
        super().__init__('detect_aruco_node')
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'image_raw',self.aruco_detect, 10)
        self.publisher = self.create_publisher(String,'num_markers',10)
        self.visualize_result = args['visualize']

        # load the ArUCo dictionary, grab the ArUCo parameters, and detect
        # the markers

        print("[INFO] detecting '{}' tags...".format(args['type']))
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args['type']])
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def aruco_detect(self, ros_image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError as e:
            print(e)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image,
                self.arucoDict, parameters=self.arucoParams)
        
        if(ids):
            print(ids)
            msg= String()
            msg.data = str(len(ids))
            self.publisher.publish(msg)

        # verify *at least* one ArUco marker was detected

        if len(corners) > 0:

            # flatten the ArUco IDs list

            ids = ids.flatten()

            # loop over the detected ArUCo corners

            for (markerCorner, markerID) in zip(corners, ids):

                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)

                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection

                cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(cv_image, topRight, bottomRight, (0, 255, 0),
                         2)
                cv2.line(cv_image, bottomRight, bottomLeft, (0, 255,
                         0), 2)
                cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)

                # draw the ArUco marker ID on the image

                cv2.putText(
                    cv_image,
                    str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    )
                print('[INFO] ArUco marker ID: {}'.format(markerID))
                
                # show the output image
                if(self.visualize_result):
                    cv2.imshow('Image', cv_image)
                    cv2.waitKey(3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-t', '--type', type=str,
                    default='DICT_ARUCO_ORIGINAL',
                    help='type of ArUCo tag to generate')
    ap.add_argument('-v', '--visualize', type=bool,
                    default=False,
                    help='visualize result')
    args, unknown = ap.parse_known_args()
    args = vars(args)
    # verify that the supplied ArUCo tag exists and is supported by OpenCV

    if ARUCO_DICT.get(args['type'], None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(args['type']))
        print('Supported Types Include:')
        print(str(ARUCO_DICT.keys()))
        sys.exit(0)

    rclpy.init(args=None)

    aruco_detect = DetectAruco(args)

    rclpy.spin(aruco_detect)
    aruco_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
