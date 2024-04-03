from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2(0)
# camera_config = picam2.create_preview_configuration()
# picam2.configure(camera_config)



#picam2.start_preview(Preview.DRM)
#picam2.start()


# picam2.capture_file("test.jpg")
# Démarrer l'aperçu

picam2.start_preview(Preview.QTGL)
picam2.start()
time.sleep(5)


# Boucle pour capturer 40 photos
for numero in range(1, 101):
    # Capturer une photo avec un nom de fichier basé sur le numéro
    nom_fichier = f"image_calibration3/cal{numero}.jpg"
    picam2.capture_file(nom_fichier)
    time.sleep(3)

picam2.close()

