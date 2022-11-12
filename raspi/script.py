import RPi.GPIO as GPIO
from grove_rgb_lcd import *
import cv2
import numpy as np
import pdf417decoder
from PIL import Image
import imutils
import math

def detect_barcode(image):
    # convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # compute the Scharr gradient magnitude representation of the images
    # in both the x and y direction
    gradX = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 1, dy = 0, ksize = -1)
    gradY = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 0, dy = 1, ksize = -1)
    # subtract the y-gradient from the x-gradient
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)
    # blur and threshold the image
    blurred = cv2.blur(gradient, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)
    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    # perform a series of erosions and dilations
    closed = cv2.erode(closed, None, iterations = 4)
    closed = cv2.dilate(closed, None, iterations = 4)
    # find the contours in the thresholded image
    (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # if no contours were found, return None
    if len(cnts) == 0:
        return None
    # otherwise, sort the contours by area and compute the rotated

    # bounding box of the largest contour
    c = sorted(cnts, key = cv2.contourArea, reverse = True)[0]
    rect = cv2.minAreaRect(c)
    box = np.int0(cv2.boxPoints(rect))

    # return the bounding box of the barcode
    return box


def calculate_rotation(box):
    # we take 2 contiguous points
    p0 = box[0]
    p1 = box[1]

    # calculate the difference in Y and X axis
    y = p1[1] - p0[1]
    x = p1[0] - p0[0]

    # calculate the inclination degree using arctan
    degrees = math.degrees(math.atan(y / x))

    # calculate the exact degrees to rotate
    degrees = math.fabs(degrees)
    if (math.fabs(x) > math.fabs(y)):
        return -degrees
    return 90 - degrees

def crop_frame(frame, box):
    # we get the upper-left and lower right corners, then add an extra 10% and crop the frame
    x1 = max(0, int(box[1, 1] * 0.9))
    x2 = int(box[3, 1] * 1.1)
    y1 = max(0, int(box[0, 0] * 0.9))
    y2 = int(box[2, 0] * 1.1)

    return frame[x1:x2, y1:y2]

def check_image(frame):
    key = None
    box = detect_barcode(frame)
    if box is None:
        print("Barcode not found")
        # Do nothing
    else:
        # if a barcode was found, draw a bounding box on the frame
        # cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        degrees = calculate_rotation(box)
        frame = crop_frame(frame, box)

        # actually rotate the image so the reader decodes the image (needs to be straight)
        try:
            frame = imutils.rotate(frame, degrees)
        except Exception:
            print("error rotating frame")

        #cv2.imshow("Frame", frame)
        #We wait 33ms (that way we have 10 fps)
        key = cv2.waitKey(100) & 0xFF

        img = Image.fromarray(frame)
        decoder = pdf417decoder.PDF417Decoder(img)
        if (decoder.decode() > 0):
            print("Barcode decoded\a\n")
            print(decoder.barcode_data_index_to_string(0))
            setRGB(0,128,64)
            setText(decoder.barcode_data_index_to_string(0))
            GPIO.output(37,1)
            time.sleep(0.15)
            GPIO.output(37,0)
            time.sleep(2)
            setRGB(0,0,0)
            textCommand(0x01)
        else:
          print(box)

    return key

#Initialize camera
cam = cv2.VideoCapture(0)

print("isOpened:" + str(cam.isOpened()))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37,GPIO.OUT)
setRGB(255,255,255)
setText("Bienvenidos")
GPIO.output(37,1)
time.sleep(0.1)
GPIO.output(37,0)
time.sleep(0.5)
GPIO.output(37,1)
time.sleep(0.1)
GPIO.output(37,0)
time.sleep(0.5)
GPIO.output(37,1)
time.sleep(0.7)
textCommand(0x01)
setRGB(0,0,0)
GPIO.output(37,0)

while (True):
    ret, image = cam.read()
    if ret:
        key = check_image(image)
        if key == ord("q"):
            break
