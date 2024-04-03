import board
import busio
import adafruit_bno055
import time
import numpy as np
from numpy.linalg import inv
#i2c = busio.I2C(board.SCL, board.SDA)


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT




i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)




# def temperature():
#     global last_val  # pylint: disable=global-statement
#     result = sensor.temperature
#     if abs(result - last_val) == 128:
#         result = sensor.temperature
#         if abs(result - last_val) == 128:
#             return 0b00111111 & result
#     last_val = result
#     return result


while True:



    # Afficher la position actuelle
    # print("Position: {}".format(position_actuelle))
        

    # print("Temperature: {} degrees C".format(sensor.temperature))
    # """
    # print(
    #     "Temperature: {} degrees C".format(temperature())
    # )  # Uncomment if using a Raspberry Pi
    # """
    # print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    # print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    #print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    # print("Quaternion: {}".format(sensor.quaternion))
    # print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    # print("Gravity (m/s^2): {}".format(sensor.gravity))
    # print()
    euler = np.radians(sensor.euler)
    R_imu_abs = np.matrix([
                                [np.cos(euler[1])*np.cos(euler[0]), np.cos(euler[1])*np.sin(euler[0]), -np.sin(euler[1])],
                                [np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0])-np.cos(euler[2])*np.sin(euler[0]),  np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0])+np.cos(euler[2])*np.cos(euler[0]),  np.sin(euler[2])*np.cos(euler[1])],
                                [np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0])+np.sin(euler[2])*np.sin(euler[0]),  np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0])-np.sin(euler[2])*np.cos(euler[0]),  np.cos(euler[2])*np.cos(euler[1])]
        ])
    
    # camera_angle = np.radians(-90)
    camera_angle = np.radians(180)
    # R_imu_camera = np.matrix([
    #                             [np.cos(camera_angle),0,-np.sin(camera_angle)],
    #                             [0, 1, 0],
    #                             [np.sin(camera_angle), 0, np.cos(camera_angle)]
    # ])
    R_imu_camera = np.matrix([
                                [1, 0, 0],
                                [0, np.cos(camera_angle), np.sin(camera_angle)],
                                [0, -np.sin(camera_angle), np.cos(camera_angle)]
    ])
    R_camera_aruco_mauvais_sens = R_imu_abs@R_imu_camera.transpose()
    R_camera_aruco = inv(R_camera_aruco_mauvais_sens)

    euler_cam = np.zeros((3,1))

    # euler_cam[0] = np.degrees(-np.arctan2(R_camera_aruco[2,1],R_camera_aruco[2,2]))
    # euler_cam[1] = np.degrees(np.arcsin(R_camera_aruco[2,0]))
    # euler_cam[2] = np.degrees(-np.arctan2(R_camera_aruco[1,0],R_camera_aruco[0,0]))

    euler_cam[0] = np.degrees(np.arctan2(R_camera_aruco[0,1],R_camera_aruco[0,0]))
    euler_cam[1] = -np.degrees(np.arcsin(R_camera_aruco[0,2]))
    euler_cam[2] = np.degrees(np.arctan2(R_camera_aruco[1,2],R_camera_aruco[2,2]))



    print("CAMERA ANGLE: {}",euler_cam)

    time.sleep(1)



