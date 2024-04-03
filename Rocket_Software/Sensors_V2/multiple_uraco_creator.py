#!/usr/bin/env python
  
'''
Welcome to the ArUco Marker Generator!
  
This program:
  - Generates ArUco markers using OpenCV and Python
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
  
# Project: ArUco Marker Generator
# Date created: 12/17/2021
# Python version: 3.8
# Reference: https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
 
desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
output_prefix = "DICT_ARUCO_ORIGINAL_id"
 
# The different ArUco dictionaries built into the OpenCV library. 
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
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
def main():
  """
  Main method of the program.
  """
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      args["type"]))
    sys.exit(0)
     
  # Load the ArUco dictionary
  #this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
  this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])

  # Get the number of markers to generate from user input
  num_markers = int(input("Enter the number of markers to generate: "))
   
  for marker_id in range(1, num_markers + 1):
    # Allocate memory for the ArUco marker
    print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(
      desired_aruco_dictionary, marker_id))
    this_marker = np.zeros((300, 300, 1), dtype="uint8")
    #cv2.aruco.drawMarker(this_aruco_dictionary, marker_id, 300, this_marker, 1)
    cv2.aruco.generateImageMarker(this_aruco_dictionary, marker_id, 300, this_marker, 1)

    # Save the ArUco tag to the current directory
    output_filename = "{}{}.png".format(output_prefix, marker_id)
    cv2.imwrite(output_filename, this_marker)
    cv2.imshow("ArUco Marker {}".format(marker_id), this_marker)
  cv2.waitKey(0)
   
if __name__ == '__main__':
  print(__doc__)
  main()
