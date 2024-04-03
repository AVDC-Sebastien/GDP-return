import board
import busio
import adafruit_bno055
import time
import numpy as np
#i2c = busio.I2C(board.SCL, board.SDA)


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT


def imu_task():

    i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # If you are going to use UART uncomment these lines
    # uart = board.UART()
    # sensor = adafruit_bno055.BNO055_UART(uart)

    last_val = 0xFFFF

    position_precedente = [0, 0, 0]  # Position initiale [x, y, z]
    vitesse_precedente = [0, 0, 0]   # Vitesse initiale [vx, vy, vz]
    acceleration_precedente = [0, 0, 0]  # Accélération initiale [ax, ay, az]
    temps_precedent = 0  # Temps au moment de la première mesure


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

        acc1 = sensor.linear_acceleration[0]  #je suis obligé de faire ca parceque sinon dans la condition if, je peux
        acc2 = sensor.linear_acceleration[1]  # avoir deux mesure different le temps que la condition soit verifiée
        acc3 = sensor.linear_acceleration[2] 
        # Mesurer le temps actuel
        temps_actuel = time.time()

        # Lire les nouvelles données d'accélération du capteur
        if (
                acc1 is not None and
                acc2 is not None and
                acc3 is not None
            ):
                acceleration_actuelle = np.array([
                    float(acc1),
                    float(acc2),
                    float(acc3)
                ])
        
        else:
                # Utiliser la valeur précédente si les données d'accélération sont manquantes
                acceleration_actuelle = np.array(acceleration_precedente)

        # Calculer la différence de temps entre les deux mesures
        dt = temps_actuel - temps_precedent
        temps_precedent = temps_actuel  # Mettre à jour le temps précédent

        # Estimation de la vitesse en fonction de l'accélération
        vitesse_actuelle = vitesse_precedente + (0.5 * (acceleration_precedente + acceleration_actuelle) * dt)

        # Estimation de la position en fonction de la vitesse
        position_actuelle = position_precedente + (0.5 * (vitesse_precedente + vitesse_actuelle) * dt)

        # Mettre à jour les valeurs précédentes pour la prochaine itération
        acceleration_precedente = acceleration_actuelle
        vitesse_precedente = vitesse_actuelle
        position_precedente = position_actuelle
            

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
        
        print("Euler angle: {}".format(sensor.euler))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        # print("Quaternion: {}".format(sensor.quaternion))
        print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        # print("Gravity (m/s^2): {}".format(sensor.gravity))
        # print()

        time.sleep(1)



