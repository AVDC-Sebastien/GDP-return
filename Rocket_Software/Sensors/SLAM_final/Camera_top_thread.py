#!/usr/bin/env python
  
'''
Welcome to the ArUco Marker Pose Estimator!
  
This program:
  - Estimates the pose of an ArUco Marker
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
from time import sleep
from picamera2 import Picamera2
import time
import logging
 
# Project: ArUco Marker Pose Estimator
# Date created: 12/21/2021
# Python version: 3.8
# Structure de données partagée


def camera_top_task(get_camera_top_meas,stop):

# Dictionary that was used to generate the ArUco marker
  aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"
  
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
  
  # Side length of the ArUco marker in meters 
  aruco_marker_side_length = 0.0785
  
  # Calibration parameters yaml file
  camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
  
  def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
        
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
        
    return roll_x, pitch_y, yaw_z # in radians
  
  #def main():
  """
  Main method of the program.
  """
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
    logging.info("[INFO] ArUCo tag is not supported")
    

  # Load the camera parameters from the saved file
  cv_file = cv2.FileStorage(
    camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
  mtx = cv_file.getNode('K').mat()
  dst = cv_file.getNode('D').mat()
  cv_file.release()

  # print(mtx)
  # print(dst)   
  # Load the ArUco dictionary
  print("[INFO] detecting '{}' markers...".format(
    aruco_dictionary_name))
  # this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
  # this_aruco_parameters = cv2.aruco.DetectorParameters_create()
  this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
  this_aruco_parameters =  cv2.aruco.DetectorParameters()
  
  # Start the video stream
  # cap = cv2.VideoCapture(0)
  cap = Picamera2(1)
  cap.start()
  sleep(2)
  n=0
  data_dict = {}
  measurement_dict = {}
  
  while(stop):
  
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    # ret, frame = cap.read() 
    # if not cap.isOpened():
    #  print("Error: Unable to open camera")
    # break
    # Capturer une image dans le flux
    # cap.capture_image("main")
    frame = cap.capture_array()
    t1 = time.time()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  # display the image on screen and wait for a keypress
  

    # Décodez le tableau NumPy en une image OpenCV
    # frame = cv2.imdecode(array, 1)

    
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      frame, this_aruco_dictionary, parameters=this_aruco_parameters)
      
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:

      # Draw a square around detected markers in the video frame
      #cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
      
      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        mtx, 
        dst)
        
      # Print the pose for the ArUco marker
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      n=n+1
      for i, marker_id in enumerate(marker_ids):
      
        # Store the translation (i.e. position) information
        transform_translation_x = tvecs[i][0][0]
        transform_translation_y = tvecs[i][0][1]
        transform_translation_z = tvecs[i][0][2]

        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        euler = r.as_euler('zxy', degrees=True)
        quat = r.as_quat()   
        
        # Quaternion format     
        transform_rotation_x = quat[0] 
        transform_rotation_y = quat[1] 
        transform_rotation_z = quat[2] 
        transform_rotation_w = quat[3] 
        
        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                      transform_rotation_y, 
                                                      transform_rotation_z, 
                                                      transform_rotation_w)
        
        roll_x = math.degrees(roll_x)
        pitch_y = math.degrees(pitch_y)
        yaw_z = math.degrees(yaw_z)

        camera_measurement = np.array([[transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z]])

        new_row = [marker_id,  # ID
               n,  # Numéro d'incrémentation
               t1,  # Temps (simulé ici)
               camera_measurement ] # Valeurs (simulées ici) converties en liste

        # Extraire l'ID et le temps de la nouvelle ligne
        current_id = new_row[0]
        incrementation = new_row[1]

        # Vérifier si l'ID existe déjà dans le dictionnaire
        if current_id not in data_dict:
            data_dict[current_id] = {}

        # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
        data_dict[current_id][incrementation] = new_row

        last_three_increments_exist = all(data_dict.get(marker_id, {}).get(n, None) is not None
                                  for n in range(n, n - 3, -1))

        # Afficher le résultat
        if last_three_increments_exist:
          if current_id not in measurement_dict:
            removed_data = measurement_dict.pop(marker_id, None)  
          measurement_dict[current_id] = {}
          measurement_dict[current_id][incrementation]= new_row
        get_camera_top_meas = measurement_dict

        # print(marker_id)
        # print("transform_translation_x: {}".format(transform_translation_x))
        # print("transform_translation_y: {}".format(transform_translation_y))
        # print("transform_translation_z: {}".format(transform_translation_z))
        # print("roll_x: {}".format(roll_x))
        # print("pitch_y: {}".format(pitch_y))
        # print("yaw_z: {}".format(yaw_z))
        # # print("roll_z: {}".format(euler[0]))
        # # print("pitch_y: {}".format(euler[1]))
        # # print("yaw_x: {}".format(euler[2]))
        # print()
        
        # Draw the axes on the marker
        #cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], aruco_marker_side_length, 1)
    
    # Display the resulting frame
    #cv2.imshow('frame',frame)
          
    # If "q" is pressed on the keyboard, 
    # exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  # Close down the video stream
  cap.release()
  # cv2.destroyAllWindows()
    
  # if __name__ == '__main__':
  #   print(__doc__)
  #   main()