import time
import cv2

def check_image(image):
    cv2.imwrite('imagen.png', image)

#Initialize camera
cam = cv2.VideoCapture(0)

for i in range(30):
  temp = cam.read()

ret, image = cam.read()


while (True):
  time.sleep(1)
  ret, image = cam.read()

  if ret:
    check_image(image)