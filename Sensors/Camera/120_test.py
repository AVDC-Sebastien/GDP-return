from picamera2 import Picamera2
from time import sleep

camera = Picamera2()
camera.start_preview()
camera.start_recording('/home/pi/Desktop/video.h264')
sleep(5)
camera.stop_recording()
camera.stop_preview()
