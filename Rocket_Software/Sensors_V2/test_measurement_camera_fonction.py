import time
import board
import adafruit_bno055
import busio
import numpy as np 
import json   
from scipy.spatial.transform import Rotation as R  




#region
i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)



#endregion    


# for i in range(1, 5):
#      print("Euler angle: {}".format(sensor.euler))
#      time.sleep(0.5)
# valeur1 = float(input("Entrez la première valeur : "))
# valeur2 = float(input("Entrez la deuxième valeur : "))
# valeur3 = float(input("Entrez la troisième valeur : "))

# target_angle_offset = (valeur1, valeur2, valeur3)
target_angle_offset = (0, 0, 0)
euler_error = 0
euler_prec = 0

while True:
    
    
    sensor_euler = sensor.euler
    angular_rate =sensor.gyro
    acceleration = sensor.linear_acceleration
    quaternions = sensor.quaternion
  
  
    
    
    heading, roll, pitch = [position - target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]
    euler = np.matrix([[heading, roll, pitch]])
    if (
            euler[0,0] is not None and
            euler[0,1] is not None and
            euler[0,2] is not None 
        ):
               
            if euler[0,0]>180:
                euler[0,0] = euler[0,0] - 360

            print("euler base", euler)
            if abs(euler_prec-euler[0,0])>140:
                 euler_error = euler_prec-euler[0,0]
            euler_prec = euler[0,0] 
            euler[0,0] = euler[0,0]-euler_error 
            if euler[0,0]>180:
                euler[0,0] = euler[0,0] - 360
            print("euler imu", euler)
            euler = np.deg2rad(euler)
       
            
    #         print("euler",euler)
    #         print("test", [[np.cos(euler[0,1])*np.cos(euler[0,0]), np.cos(euler[0,1])*np.sin(euler[0,0]), -np.sin(euler[0,1])],
    #                                     [np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]),  np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])+np.cos(euler[0,2])*np.cos(euler[0,0]),  np.sin(euler[0,2])*np.cos(euler[0,1])],
    #                                     [np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])+np.sin(euler[0,2])*np.sin(euler[0,0]),  np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.sin(euler[0,2])*np.cos(euler[0,0]),  np.cos(euler[0,2])*np.cos(euler[0,1])]
    #  ] )
            R_imu_abs = np.matrix([
                                        [np.cos(euler[0,1])*np.cos(euler[0,0]), np.cos(euler[0,1])*np.sin(euler[0,0]), -np.sin(euler[0,1])],
                                        [np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]),  np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])+np.cos(euler[0,2])*np.cos(euler[0,0]),  np.sin(euler[0,2])*np.cos(euler[0,1])],
                                        [np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])+np.sin(euler[0,2])*np.sin(euler[0,0]),  np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.sin(euler[0,2])*np.cos(euler[0,0]),  np.cos(euler[0,2])*np.cos(euler[0,1])]
        ])    
            r = R.from_euler('zyx', [euler[0,0], euler[0,1], euler[0,2]], degrees=True)  
            
            # R_imu_abs = r.as_matrix()
            offset_angle = 180
            # camera_angle = np.radians(-90)
            alpha = np.radians(offset_angle)
            # R_imu_camera = np.matrix([
            #                             [np.cos(camera_angle),0,-np.sin(camera_angle)],
            #                             [0, 1, 0],
            #                             [np.sin(camera_angle), 0, np.cos(camera_angle)]
            # ])
            R_imu_camera = np.matrix([
                                        [1, 0, 0],
                                        [0, np.cos(alpha), np.sin(alpha)],
                                        [0, -np.sin(alpha), np.cos(alpha)]
            ])
            # R_imu_camera = np.matrix([
                                        
            #                             [ np.cos(alpha), np.sin(alpha),0],
            #                             [-np.sin(alpha), np.cos(alpha), 0],
            #                             [0, 0, 1]
            # ])
            R_camera_aruco_mauvais_sens = R_imu_abs@R_imu_camera.transpose()
            R_camera_aruco = np.linalg.inv(R_camera_aruco_mauvais_sens)
            # R_camera_aruco = R_imu_abs
            # print("diff", R_imu_abs-r)

            #euler_cam = np.zeros((3,1))

            # euler_cam[0] = np.degrees(-np.arctan2(R_camera_aruco[2,1],R_camera_aruco[2,2]))
            # euler_cam[1] = np.degrees(np.arcsin(R_camera_aruco[2,0]))
            # euler_cam[2] = np.degrees(-np.arctan2(R_camera_aruco[1,0],R_camera_aruco[0,0]))

            euler_cam_z = np.degrees(np.arctan2(R_camera_aruco[0,1],R_camera_aruco[0,0]))
            euler_cam_y = -np.degrees(np.arcsin(R_camera_aruco[0,2]))
            euler_cam_x = -np.degrees(np.arctan2(R_camera_aruco[1,2],R_camera_aruco[2,2]))

            

           

            
            
            # print("state markers",state_markers)

        

            estimation_2 = np.matrix([
                 [ euler_cam_z, euler_cam_y, euler_cam_x]
             ])

            # print("estimation des mesure de la cam", estimation.transpose())
            print("estimation des mesure de la cam", estimation_2.transpose())
            
            time.sleep(2)
                
