import board
import busio
import adafruit_bno055
import time
import numpy as np
#i2c = busio.I2C(board.SCL, board.SDA)


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT




i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# The target angle offset used to reorient the sensor
# (heading, roll, pitch)
target_angle_offset = (0, 0, 0)

# The project's main while loop
while True:
    # Get the Euler angle values from the sensor
    # The Euler angle limits are: +180 to -180 pitch, +360 to -360 heading, +90 to -90 roll
    sensor_euler = sensor.euler
    print(f"Euler angle: {sensor_euler}")
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        # print("Quaternion: {}".format(sensor.quaternion))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    # Adjust the Euler angle values with the target_angle_offset
    heading, roll, pitch = [position - target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]

    # print(heading)
    # print(roll)
    # print(pitch)
    time.sleep(2)

    # # Scale the heading for horizontal movement range
    # horizontal_mov = int(map_range(heading, -20, 20, -30, 30))
    # print(f"mouse x: {horizontal_mov}")

    # # Scale the roll for vertical movement range
    # vertical_mov = int(map_range(roll, -25, 25, 30, 30))
    # print(f"mouse y: {vertical_mov}")

    # # Translate to stuff needed for HID
    # mouse.move(x=horizontal_mov)
    # mouse.move(y=vertical_mov)
    
    # # Check the "reorient" button was pressed
    # if reorientation_button:
    #     print(f"Reorient the sensor")
    #     # Use the current Euler angle values to reorient the target angle
    #     target_angle_offset = [angle for angle in sensor_euler]