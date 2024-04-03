import RPi.GPIO as GPIO
import time
import json

GPIO.setmode(GPIO.BCM)
lidar_dict = {}

# print "+-----------------------------------------------------------+"
# print "|   Mesure de distance par le capteur ultrasonore HC-SR04   |"
# print "+-----------------------------------------------------------+"

Trig = 23          # Entree Trig du HC-SR04 branchee au GPIO 23
Echo = 24         # Sortie Echo du HC-SR04 branchee au GPIO 24

GPIO.setup(Trig,GPIO.OUT)
GPIO.setup(Echo,GPIO.IN)

GPIO.output(Trig, False)

# repet = input("Entrez un nombre de repetitions de mesure : ")

for x in range(150):    # On prend la mesure "repet" fois
   print(x)
   import time
   time.sleep(0.2)       # On la prend toute les 1 seconde

   GPIO.output(Trig, True)
   time.sleep(0.00001)
   GPIO.output(Trig, False)

   while GPIO.input(Echo)==0:  ## Emission de l'ultrason
     debutImpulsion = time.time()

   while GPIO.input(Echo)==1:   ## Retour de l'Echo
     finImpulsion = time.time()

   t1 = time.time()
   distance = round((finImpulsion - debutImpulsion) * 340 * 100 / 2, 1)  ## Vitesse du son = 340 m/s

   new_row = [ 
            t1,  # Temps (simulé ici)
            distance ] # Valeurs (simulées ici) converties en liste

      # Extraire l'ID et le temps de la nouvelle ligne
   time = new_row[0]


      # Vérifier si l'ID existe déjà dans le dictionnaire
   if time not in lidar_dict:
          lidar_dict[time] = {}

      # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
   lidar_dict[time] = new_row
nom_fichier = 'lidar_data.json'

# Ouverture du fichier en mode écriture pour créer le fichier s'il n'existe pas
with open(nom_fichier, 'w') as fichier:
    # Écriture du contenu du dictionnaire dans le fichier au format JSON
    json.dump(lidar_dict, fichier)
GPIO.cleanup()