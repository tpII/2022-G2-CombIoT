import time
import cv2
import matplotlib.pyplot as plt

def check_image(image):



    
cam = cv2.VideoCapture(0)

while (True):
    result, image = cam.read()
    check_image(image)
    sleep(1)
