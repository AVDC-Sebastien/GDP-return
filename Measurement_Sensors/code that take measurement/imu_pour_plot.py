import time
import board
import adafruit_bno055
import busio
import numpy as np 
import json      
#region
i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

imu_dict = {}

#endregion    


for i in range(1, 5):
     print("Euler angle: {}".format(sensor.euler))
     time.sleep(0.5)
valeur1 = float(input("Entrez la première valeur : "))
valeur2 = float(input("Entrez la deuxième valeur : "))
valeur3 = float(input("Entrez la troisième valeur : "))

target_angle_offset = (valeur1, valeur2, valeur3)
t_ini = time.time()
t_boucle = t_ini
while t_boucle-t_ini < 60:
    print(t_boucle-t_ini)
    t1 = time.time()
    sensor_euler = sensor.euler
    angular_rate =sensor.gyro
    acceleration = sensor.linear_acceleration

    # Adjust the Euler angle values with the target_angle_offset
    
    heading, roll, pitch = [position - target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]
    euler = np.array([[heading, roll, pitch]])
    if (
            euler[0][0] is not None and
            euler[0][1] is not None and
            euler[0][2] is not None and
            angular_rate[0] is not None and
            angular_rate[1] is not None and
            angular_rate[2] is not None and
            acceleration[0] is not None and
            acceleration[1] is not None and
            acceleration[2] is not None 
        ):
            
            if euler[0][0]>180:
                euler[0][0] = euler[0][0] - 360
            get_imu_meas = [
                            [euler[0][0], euler[0][1], euler[0][2]],
                            [angular_rate[0], angular_rate[1], angular_rate[2]],
                            [acceleration[0], acceleration[1], acceleration[2]]
                            ]
            new_row = [ 
                    t1,  # Temps (simulé ici)
                    get_imu_meas ] # Valeurs (simulées ici) converties en liste

            # Extraire l'ID et le temps de la nouvelle ligne
            time = new_row[0]


          # Vérifier si l'ID existe déjà dans le dictionnaire
            if time not in imu_dict:
                imu_dict[time] = {}

          # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
            imu_dict[time] = new_row
            
    import time
    time.sleep(0.01)
    t_boucle = time.time()

nom_fichier = 'imu_data.json'

# Ouverture du fichier en mode écriture pour créer le fichier s'il n'existe pas
with open(nom_fichier, 'w') as fichier:
    # Écriture du contenu du dictionnaire dans le fichier au format JSON
    json.dump(imu_dict, fichier)