import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Initialisation de la caméra Pi
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera)

# Attendre que la caméra soit prête
time.sleep(0.1)

# Initialisation du détecteur de flux optique
previous_frame = None

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculer le flux optique à partir de l'image actuelle et de l'image précédente
    if previous_frame is not None:
        flow = cv2.calcOpticalFlowFarneback(previous_frame, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        
        # Accéder aux vecteurs de vitesse du flux optique
        u = flow[..., 0]  # Composante horizontale
        v = flow[..., 1]  # Composante verticale

        # Faire quelque chose avec les vecteurs de vitesse (par exemple, les afficher)
        print("Vecteur de vitesse horizontal (u):")
        print(u)
        print("Vecteur de vitesse vertical (v):")
        print(v)

    # Mettre à jour l'image précédente
    previous_frame = gray

    # Effacer le tampon pour la prochaine image
    rawCapture.truncate(0)

    # Interruption du processus après un certain temps (par exemple, 10 secondes)
    if time.time() > start_time + 10:
        break

# Fermer la caméra Pi
camera.close()
