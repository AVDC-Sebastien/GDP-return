import board
import adafruit_bno055


i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

while True:
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print()