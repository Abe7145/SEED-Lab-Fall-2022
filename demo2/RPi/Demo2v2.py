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

#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()

abus = smbus.SMBus(1)
address = 0x20

data = ''

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

def displayLCD(flag, angle, distance):
    global data
    if flag == True:
#        try:
#            if data != '123':
#                print('yo')
#                abus.write_byte(address, angle)
##            if abs(angle) > 3:
##                abus.write_byte(address, angle)
##            elif abs(angle) < 3:
##                abus.write_byte(address, (distance * 100) )
#        except:
#            print('io error')
        try:
            if data != '123':
                data = ''
                for i in range(0,3):
                    data += chr(abus.read_byte(address))
                print(data)
            #Thing to consider possibly: Might want to slow transmission of values to synchronize with Arduino sampling time?
            if data == '123':
                #print('hi')
                abus.write_byte(address, int(distance * 10) )
                print('send')
                data = ''
            else:
                abus.write_byte(address, angle)
        except:
            None
            #print('work')
            #lcd.message = str(angle) + ' deg'
            #if counter == 1:
                #lcd.message = 'Marker found! \nAngle:' + str(angle) + ' deg'
            #print(angle)
            #lcd.clear()
    else:
        print('fail')
        #lcd.message = 'Marker found! \nAngle:' + str(angle) + ' deg'

        #lcd.message = 'No marker \nfound.'
    return -1

#npzfile = np.load('calibrationvalues.npz')

# Start the camera
camera = cv2.VideoCapture(0)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 2);
camera.set(cv2.CAP_PROP_FPS, 20);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25);
camera.set(cv2.CAP_PROP_EXPOSURE, 40);

markerSize = 5.08 # in cm, 5.39 for demo1 marker, 5.08 for the one we printed out, IMPORTANT: CHANGE ON DAY OF DEMO
# 0.058
# Start the cleaning thread
cam_cleaner = CameraBufferCleanerThread(camera)

counter = 0

# Takes in the calibration file from separate calibration program
# Credit to Kyle Bersani on Github for calibration and import technique
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
#Rename to what we use for estimating the distance
mtx = cameraMatrix
dist = distCoeffs
flag = False
# Use the frame whenever you want
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
        if len(corners)> 0:
            flag = True # Flag to print to LCD
            counter += 1 # Counter to determine when to send information
            ids = ids.flatten() # flatten ID list
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSize, mtx, dist) # Find the rotational and translational
                                                                                            # vectors to get distance and angle?
            #print(tvec)                                                                                         
            # loop over the marker corners
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
                # Calculate the angle from camera to the object by using the provided formula, 53.76 == FOV of CAMERA
                actAngle = round( (53.76/2) * ( (xCoord - cX) / xCoord))
                z = (tvec[0][0][2]/100)
                y = (tvec[0][0][0]/100)
                # Calculate the distance using z-vector of tvec and compensate for angle using trig, divide by 100 to get ft
                distFromCamToMarker = round( math.sqrt(math.pow(z,2) + math.pow(y,2)) , 1) #* math.cos(actAngle * (math.pi/ 180)) 
                # Display the ID's and angle
                #anglePrint = str(angle) + ' degrees'
                actPrint = str(actAngle) + ' degrees'
                distPrint = str(distFromCamToMarker) + ' ft'
                print(distPrint)
                #cv2.putText(img, distPrint, (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                cv2.putText(img, actPrint, (topRight[0], topRight[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                #writeNumber(angle)
                #if counter <= 20:
                displayLCD(flag, actAngle, distFromCamToMarker)
            
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

