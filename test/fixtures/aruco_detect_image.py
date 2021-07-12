import argparse
import cv2
import sys
import imutils



ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class ArucoDetect:
    
    def __init__(self):
        self.parse_args()
        self.detectTag()


    def parse_args(self):
        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-i", "--image", required=True,help="path to input image containing ArUCo tag")
        ap.add_argument("-t", "--type", type=str,default="DICT_ARUCO_ORIGINAL",help="type of ArUCo tag to detect")
        args = vars(ap.parse_args())

        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        if ARUCO_DICT.get(args["type"], None) is None:
	        print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
	        sys.exit(0)

        # load the input image from disk and resize it
        print("[INFO] loading image...")
        self.image = cv2.imread(args["image"])
        self.image = imutils.resize(self.image, width=600)

        # add a white border to the image, helps with detection
        self.image = cv2.copyMakeBorder(self.image,20,20,20,20,cv2.BORDER_CONSTANT,value=[255,255,255])

        print("[INFO] detecting '{}' tags...".format(args["type"]))
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
        cv2.imshow("Image", self.image)
        cv2.waitKey(0)

    def detectTag(self):
        # load the ArUCo dictionary, grab the ArUCo parameters, and detect
        # the markers
     
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.image, self.arucoDict,parameters=arucoParams)
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
                cv2.line(self.image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(self.image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                # show the output image
                cv2.imshow("Image", self.image)
                cv2.waitKey(0)

if __name__ == '__main__':
    ar_detect = ArucoDetect()
