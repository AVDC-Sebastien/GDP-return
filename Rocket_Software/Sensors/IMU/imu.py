import board
import adafruit_bno055
import imufusion
import numpy as np
# sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED

# Activate I2C

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# while True:
#     print(sensor.acceleration)
#     print(sensor.gyro)


ahrs = imufusion.Ahrs()
while True:
    try:
        gyro = np.asarray(sensor.gyro)
        accel = np.asarray(sensor.acceleration)
        magnet =np.asarray(sensor.magnetic)
        ahrs.update(gyro, accel ,magnet, 1 / 1000)  # 100 Hz sample rate
        print(ahrs.quaternion.to_euler())
    except Exception as e:
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print(sensor.magnetic)
        print(e)
        break