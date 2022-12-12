import RPi.GPIO as GPIO
from grove_rgb_lcd import *
import cv2
import numpy as np
import pdf417decoder
from PIL import Image
import imutils
import math
import time
import json


def detect_barcode(image):
    # convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # compute the Scharr gradient magnitude representation of the images
    # in both the x and y direction
    gradX = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
    gradY = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=-1)
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
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    # find the contours in the thresholded image
    (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if no contours were found, return None
    if len(cnts) == 0:
        return None
    # otherwise, sort the contours by area and compute the rotated

    # bounding box of the largest contour
    c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
    rect = cv2.minAreaRect(c)
    box = np.int0(cv2.boxPoints(rect))

    # return the bounding box of the barcode
    return box


def calculate_rotation(box):
    p0 = box[0]; p0x = p0[0]; p0y = p0[1]
    p1 = box[1]; p1x = p1[0]; p1y = p1[1]
    p2 = box[2]; p2x = p2[0]; p2y = p2[1]
    p3 = box[3]; p3x = p3[0]; p3y = p3[1]

    # we take the 2 contiguous points p2 & p1 that match the condition: p2x>p1x && p2y<p2x
    if p1x > p0x and p1y < p0y:
        p2 = p1
        p1 = p0
        p0 = p3
    elif p3x > p2x and p3y < p2y:
        p0 = p1
        p1 = p2
        p2 = p3

    # calculate the difference in Y and X axis
    y = p1[1] - p2[1]
    x = p2[0] - p1[0]

    # calculate the inclination degree using arctan
    if y == 0 or x == 0:
        return 0
    degrees = math.degrees(math.atan(y / x))

    if (p2[0] - p1[0]) < (p0[1] - p1[1]):
        return 90 + degrees
    return degrees


def crop_frame(frame, box):
    # we get the upper-left and lower right corners, then add an extra 10% and crop the frame
    x1 = max(0, int(box[1, 1] * 0.9))
    x2 = int(box[3, 1] * 1.1)
    y1 = max(0, int(box[0, 0] * 0.9))
    y2 = int(box[2, 0] * 1.1)

    return frame[x1:x2, y1:y2]


def emit_sound(secs):
    # We turn the buzzer, wait for secs seconds, and then turn it off
    GPIO.output(BUZZER_PIN, 1)
    time.sleep(secs)
    GPIO.output(BUZZER_PIN, 0)


def is_valid_format(obj):
    return "nombre" in obj and "apellido" in obj and "DNI" in obj and "fecha" in obj and "hora" in obj and "email" in obj


def is_valid_date(obj):
    # Parse ticket date
    date = time.strptime(obj["fecha"] + " " + obj["hora"], "%Y-%m-%d %H:%M:%S")

    # Get the limit dates for which the ticket is valid (1 hour before, 2 hours later)
    bottom_date = time.localtime(time.time() - 3600)
    top_date = time.localtime(time.time() + 3600 * 2)

    return bottom_date < date < top_date


def get_pretty_ticket_data(obj):
    name = obj["apellido"] + ", " + obj["nombre"]
    date_str = obj["fecha"] + " " + obj["hora"]

    return name[0:16] + "\n" + date_str[0:16]


def check_decoded_code(code):
    # Replace single quotes for double quotes (json is generated malformed)
    code = code.replace("'", '"')

    try:
        obj = json.loads(code)
    except json.decoder.JSONDecodeError:
        obj = json.loads("{}")

    # Check if json format is valid
    if not is_valid_format(obj):
        sounds = 4
        setRGB(255, 0, 0)  # red
        setText("Formato invalido")
    else:
        setText(get_pretty_ticket_data(obj))

        if is_valid_date(obj):
            sounds = 1
            # Ticket is valid
            setRGB(0, 128, 64)  # green
        else:
            sounds = 3
            # Ticket is out of range
            setRGB(255, 255, 0)  # yellow

    # We make small sounds (1 means OK, 3 means out of range, 4 means wrong format)
    for i in range(sounds):
        emit_sound(0.15)
        time.sleep(0.15)

    # After 3 seconds, we clear the screen and turn it off
    time.sleep(3)
    setRGB(0, 0, 0)
    textCommand(0x01)


def check_image(frame):
    key = None
    box = detect_barcode(frame)
    if box is None:
        # If no barcode is found in the image, do nothing until next frame
        print("Barcode not found")
    else:
        degrees = calculate_rotation(box)

        # actually rotate the image so the reader decodes the image (needs to be straight)
        try:
            frame = imutils.rotate(frame, degrees)
        except Exception:
            print("error rotating frame")

        # We detect the barcode pattern with the new position
        box = detect_barcode(frame)
        if box is None:
            # It should not happen, but just in case we check if barcode pattern was not found
            return key
        frame = crop_frame(frame, box)

        # We wait 100ms (that way we have 10 fps)
        time.sleep(0.1)

        # We load the image as an Image object
        img = Image.fromarray(frame)
        # Actually try to decode the pdf417 code
        decoder = pdf417decoder.PDF417Decoder(img)
        if decoder.decode() > 0:
            print("Barcode decoded\a\n")
            code = decoder.barcode_data_index_to_string(0)
            check_decoded_code(code)
        else:
            print(box)

    return key


def welcome_message():
    setRGB(255, 255, 255)
    setText("  Bienvenidos!  ")
    for i in range(2):
        emit_sound(0.1)
        time.sleep(0.5)
    emit_sound(0.7)
    textCommand(0x01)
    setRGB(0, 0, 0)


BUZZER_PIN = 37

# Initialize camera
cam = cv2.VideoCapture(0)

print("isOpened:" + str(cam.isOpened()))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

welcome_message()

while True:
    ret, image = cam.read()
    if ret:
        check_image(image)