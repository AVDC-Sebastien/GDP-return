# SPDX-FileCopyrightText: 2023 JG for Cedar Grove Maker Studios
# SPDX-License-Identifier: MIT

"""
`bno055_calibrator.py`
===============================================================================
A CircuitPython module for calibrating the BNo055 9-DoF sensor. After manually
calibrating the sensor, the module produces calibration offset tuples for use
in project code.

* Author(s): JG for Cedar Grove Maker Studios

Implementation Notes
--------------------
**Hardware:**
* Adafruit BNo055 9-DoF sensor
**Software and Dependencies:**
* Driver library for the sensor in the Adafruit CircuitPython Library Bundle
* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
"""
def imu_calibration(self):
    import time
    import board
    import adafruit_bno055
    import busio
    import numpy as np


    # pylint: disable=too-few-public-methods
    class Mode:
        CONFIG_MODE = 0x00
        ACCONLY_MODE = 0x01
        MAGONLY_MODE = 0x02
        GYRONLY_MODE = 0x03
        ACCMAG_MODE = 0x04
        ACCGYRO_MODE = 0x05
        MAGGYRO_MODE = 0x06
        AMG_MODE = 0x07
        IMUPLUS_MODE = 0x08
        COMPASS_MODE = 0x09
        M4G_MODE = 0x0A
        NDOF_FMC_OFF_MODE = 0x0B
        NDOF_MODE = 0x0C


    # Uncomment these lines for UART interface connection
    # uart = board.UART()
    # sensor = adafruit_bno055.BNO055_UART(uart)

    # Instantiate I2C interface connection
    # i2c = board.I2C()  # For board.SCL and board.SDA
    i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    sensor.mode = Mode.NDOF_MODE  # Set the sensor to NDOF_MODE

    print("Magnetometer: Perform the figure-eight calibration dance.")
    while not sensor.calibration_status[3] == 3:
        # Calibration Dance Step One: Magnetometer
        #   Move sensor away from magnetic interference or shields
        #   Perform the figure-eight until calibrated
        print(f"Mag Calib Status: {100 / 3 * sensor.calibration_status[3]:3.0f}%")
        time.sleep(1)
    print("... CALIBRATED")
    time.sleep(1)

    print("Accelerometer: Perform the six-step calibration dance.")
    while not sensor.calibration_status[2] == 3:
        # Calibration Dance Step Two: Accelerometer
        #   Place sensor board into six stable positions for a few seconds each:
        #    1) x-axis right, y-axis up,    z-axis away
        #    2) x-axis up,    y-axis left,  z-axis away
        #    3) x-axis left,  y-axis down,  z-axis away
        #    4) x-axis down,  y-axis right, z-axis away
        #    5) x-axis left,  y-axis right, z-axis up
        #    6) x-axis right, y-axis left,  z-axis down
        #   Repeat the steps until calibrated
        print(f"Accel Calib Status: {100 / 3 * sensor.calibration_status[2]:3.0f}%")
        time.sleep(1)
    print("... CALIBRATED")
    time.sleep(1)

    print("Gyroscope: Perform the hold-in-place calibration dance.")
    while not sensor.calibration_status[1] == 3:
        # Calibration Dance Step Three: Gyroscope
        #  Place sensor in any stable position for a few seconds
        #  (Accelerometer calibration may also calibrate the gyro)
        print(f"Gyro Calib Status: {100 / 3 * sensor.calibration_status[1]:3.0f}%")
        time.sleep(1)
    print("... CALIBRATED")
    time.sleep(1)

    print("\nCALIBRATION COMPLETED")
    print("Insert these preset offset values into project code:")
    print(f"  Offsets_Magnetometer:  {sensor.offsets_magnetometer}")
    print(f"  Offsets_Gyroscope:     {sensor.offsets_gyroscope}")
    print(f"  Offsets_Accelerometer: {sensor.offsets_accelerometer}")

    reponse = input("Do you want to set an offset for the imu? ? (y/n) : ").lower()
        
    if reponse == "y":
       
        temps_debut = time.time()
        satisfaction = "n"

        # Exécuter la tâche pendant 10 secondes
        while (time.time() - temps_debut) < 10:
            print("Euler angle: {}".format(sensor.euler))

        while satisfaction == "n":
            yaw = float(input("give the wanted yaw offset : "))
            pitch = float(input("give the wanted pitch offset : "))
            roll = float(input("give the wanted roll offset : "))
            target_angle_offset = (yaw, pitch, roll)
            while (time.time() - temps_debut) < 10:
                print("Euler angle: {}".format(sensor.euler))

            reponse_offset = input("are you satisfied of the offset? ? (y/n) : ").lower()
        
            if reponse_offset == "y":
                satisfaction = "y"
                self.Start_sensors()
   
    elif reponse == "n":
        target_angle_offset = (0, 0, 0)
        self.Start_sensors()
