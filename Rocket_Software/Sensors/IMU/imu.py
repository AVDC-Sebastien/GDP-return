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

gyro_store_data = np.zeros(3)
accel_store_data = np.zeros(3)
magne_store_data = np.zeros(3)

ahrs = imufusion.Ahrs()

while True:
    try:
        gyro = np.array([float(sensor.gyro[0]),float(sensor.gyro[1]),float(sensor.gyro[2])])
        accel = np.array([float(sensor.acceleration[0]),float(sensor.acceleration[1]),float(sensor.acceleration[2])])
        magnet = np.array([float(sensor.magnetic[0]),float(sensor.magnetic[1]),float(sensor.magnetic[2])])
        gyro_store_data = gyro
        accel_store_data = accel
        magne_store_data = magnet
    except:
        gyro = gyro_store_data
        accel = accel_store_data
        magnet = magne_store_data
    
    try:
        ahrs.update(gyro, accel ,magnet, 1 / 1000)  # 100 Hz sample rate
        print(ahrs.quaternion.to_euler())
    except Exception as e:
        print(sensor.magnetic)
        print(e)
        break