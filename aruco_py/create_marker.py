import numpy as np
import argparse
import cv2
import sys

"""
Implementation based on https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
"""

# import list of aruco tags from OpenCV
from .markers.tag_definitions import ARUCO_DICT

class GenerateTag:
    def generate(self):
        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-o", "--output", required=True,help="path to output image containing ArUCo tag")
        ap.add_argument("-i", "--id", type=int, required=True, help="ID of ArUCo tag to generate")
        ap.add_argument("-r", "--res", type=int, default=200, help="Resolution of Image eg. 200x200")
        ap.add_argument("-b", "--border", type=int, default=1, help="Border bits” to pad the tag with")
        ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",help="type of ArUCo tag to generate")
        args = vars(ap.parse_args())

        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        if ARUCO_DICT.get(args["type"], None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported\n".format(args["type"]))
            print("Supported Types Include:")
            print(str(ARUCO_DICT.keys()))
            sys.exit(0)

        #TODO verify that the resolution is large enough for the desired tag size


        # load the ArUCo dictionary
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])

        tag = np.zeros((args["res"], args["res"], 1), dtype="uint8")
        cv2.aruco.drawMarker(arucoDict, args["id"], args["res"], tag, args["border"])

        # write the generated ArUCo tag to disk
        cv2.imwrite(args["output"], tag)

def main():

    gen_tag = GenerateTag()
    gen_tag.generate()
    print("Tag Generated")



