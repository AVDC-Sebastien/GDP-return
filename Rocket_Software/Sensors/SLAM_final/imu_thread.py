import board
import adafruit_bno055
import time
import numpy as np
#i2c = busio.I2C(board.SCL, board.SDA)


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT



def imu_task(get_imu_meas, target_angle_offset,stop):

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


    while not stop:
        t = time.time()
        sensor_euler = sensor.euler
        angular_rate =sensor.gyro
        acceleration = sensor.linear_acceleration
        # Adjust the Euler angle values with the target_angle_offset
        heading, roll, pitch = [position - target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]
        euler = np.array([[heading, roll, pitch]])
        if (
                euler[0] is not None and
                euler[1] is not None and
                euler[2] is not None and
                angular_rate[0] is not None and
                angular_rate[1] is not None and
                angular_rate[2] is not None and
                acceleration[0] is not None and
                acceleration[1] is not None and
                acceleration[2] is not None 
            ):
                get_imu_meas = [
                                [t],
                                [euler[0], euler[1], euler[2]],
                                [angular_rate[0], angular_rate[1], angular_rate[2]],
                                [acceleration[0], acceleration[1], acceleration[2]]
                                ]
        # shared_data["imu_measurement"] = imu_data


       



