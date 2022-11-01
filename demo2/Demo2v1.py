# SEEDsters
# Demo 2 CV Code

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import numpy as np
import sys
import math
import smbus
import smbus2
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import pickle
import os


lcd_columns = 16
lcd_rows = 2

i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()

abus = smbus.SMBus(1)
address = 0x20

# Define the thread that will continuously pull frames from the camera
class CameraBufferCleanerThread(threading.Thread):
    def __init__(self, camera, name='camera-buffer-cleaner-thread'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferCleanerThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()

def displayLCD(flag, angle):
    if flag == True:
        try:
            #defAngle = 360
            #abus.write_byte(address,angle)
            #lcd.message = str(angle) + ' deg'
            if counter == 1:
                lcd.message = 'Marker found! \nAngle:' + str(angle) + ' deg'
            #print(angle)
            #lcd.clear()
        except:
            None
    else:
        print('io error')
        #lcd.message = 'Marker found! \nAngle:' + str(angle) + ' deg'

        #lcd.message = 'No marker \nfound.'
    return -1

npzfile = np.load('calibrationvalues.npz')

# Start the camera
camera = cv2.VideoCapture(0)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 2);
camera.set(cv2.CAP_PROP_FPS, 20);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25);
camera.set(cv2.CAP_PROP_EXPOSURE, 40);

markerSize = 5.59 # in cm
# Start the cleaning thread
cam_cleaner = CameraBufferCleanerThread(camera)

counter = 0

if not os.path.exists('./calibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration.pckl', 'rb')
    (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()
# Use the frame whenever you want
mtx = cameraMatrix
dist = distCoeffs
flag = False
while True:
    if cam_cleaner.last_frame is not None:
        img = cv2.cvtColor(cam_cleaner.last_frame, cv2.COLOR_BGR2GRAY)
        # (DISTORTION STUFF)
        # calibrate with chessboard 
#        h, w = img.shape[:2]
#        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(npzfile['old'], npzfile['dist'], (w,h), 1, (w,h))
#        img = cv2.undistort(img, npzfile['old'], npzfile['dist'], None, npzfile['new']) 
#        # crop the image  
#        x,y,w,h = roi
#        img = img[y:y+h, x:x+w]
        # Obtain the image pixel size, and find the horizontal (x-coordinate) center
        imgSize = img.shape
        xCoord = (imgSize[1] / 2)
        yCoord = (imgSize[0] / 2)
        # Import the dictonary of markers, define the parameters, and load values into vars
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
        # Verify if an ArUco marker is detected
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSize, mtx, dist)
        if len(corners)> 0:
            flag = True
            # flatten ID list
            counter += 1
            ids = ids.flatten()
            # loop over the marker coners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract marker corners and convert them to integers
                corners = markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                #draw bounding box
                cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                #Find necessary vectors
                # Create a marker size constant
                # Calculate the angle from camera to the object by using the provided formula
                actAngle = round( (57/2) * ( (xCoord - cX) / xCoord), 1)
                # Radians
                print(tvec[0][0][2]/100)
                #actAngle = actAngle * (math.pi / 180)
                 # Use pixel manipulation to determine quadrants, then send the angle corresponding to each quadrant to the arduino
                if cX > xCoord and cY < yCoord:
                    angle = 1
                elif cX > xCoord and cY > yCoord:
                    angle = 2
                elif cX < xCoord and cY > yCoord:
                    angle = 3
                elif cX < xCoord and cY < yCoord:
                    angle = 0
                # Display the ID's and angle
                anglePrint = str(angle) + ' degrees'
                actPrint = str(actAngle) + ' degrees'
                #cv2.putText(img, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                cv2.putText(img, actPrint, (topRight[0], topRight[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                #writeNumber(angle)
                #if counter <= 20:
                displayLCD(flag, actAngle)
                
#            print(angle)
#            print(actAngle)
            #currentPos = abus.read_byte(slave_address)
#           readNumber()
            #readLCD(angle)
        # show the output image
        cv2.imshow('Image', img)
#    else:
#        cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
cv2.destroyAllWindows()

