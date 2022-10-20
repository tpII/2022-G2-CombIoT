import cv2

#Initialize camera (with this object we control the web camera)
cam = cv2.VideoCapture(0)

#We check if web camera was detected and opened successfully
if (cam.isOpened()):
  success, image = cam.read()

  #We check if image was captured OK
  if (success):
    print("Image captured successfully")
    #We store the picture in the filesystem
    cv2.imwrite('image-dark.png', image)

else:
  print("Camera wasn't detected. Exiting program")
