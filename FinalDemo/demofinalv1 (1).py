# SEEDsters
# Final Demo 

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
#import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import pickle
import os
import pygame


#lcd_columns = 16
#lcd_rows = 2

#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()

abus = smbus.SMBus(1)
address = 0x20

music_play = 1

#GLOBALS
#global data 
#global nextMarker

data = ''
data2 = ''
nextMarker = 0

seemarker = False
currentseemarker = False
pastseemarker = False

iterate = 0 # iterate sequence


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
    global iterate
    global data
    global nextMarker
    if flag == True:
#        print(distance)
#        if distance < 0.6:
#            print('yo')
#            nextMarker = 1
        try:
            if '1' not in data and '2' not in data and '3' not in data: #'1' not in data and '2' not in data and '3' not in data:
                #print('hi')
                data = ''
                for i in range(0,3):
                    #print('test')
                    data += chr(abus.read_byte(address))
                print(data)
            #Thing to consider possibly: Might want to slow transmission of values to synchronize with Arduino sampling time?
            if '1' in data and '2' in data and '3' in data:
                iterate = 1
                #print('yo')
                #data2 = ''
                print(distance)
                abus.write_byte(address, int(distance * 10) )
                #print('send')
                data = ''
                
            else:
                #print('hello')
                print(angle)
                abus.write_byte(address, angle)
                #print('hu')
                
        except:
            None
    else:
        print('fail')
    return -1

# Start the camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 2);
camera.set(cv2.CAP_PROP_FPS, 20);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25);
camera.set(cv2.CAP_PROP_EXPOSURE, 1);

markerSize = 5.00 # in cm, 5.39 for demo1 marker, 5.08 for the one we printed out, IMPORTANT: CHANGE ON DAY OF DEMO # 4.5 cm

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
pygame.mixer.init()
pygame.mixer.music.load('OUTRO.wav')

guidance = [5,4,3,2,1,0, -1] # sequence
index = 0 # Initialize sequence
#pygame.mixer.music.play(-1)


while True:
    if iterate == 1:
        try:
            data2 = ''
            for i in range(0,3):
            
                data2 += chr(abus.read_byte(address))
            if ( data2 != '0x0' ):
                print(data)
                print(data2)
                print('')
        except:
            None
        
    if '4' in data2 and '5' in data2 and '6' in data2:
        #print('yo')
        abus.write_byte(address, int(100) )
        nextMarker = 1
        data2 = ''
        iterate = 0
    
    if cam_cleaner.last_frame is not None:
        #print(nextMarker)
        img = cv2.cvtColor(cam_cleaner.last_frame, cv2.COLOR_BGR2GRAY)
        # Obtain the image pixel size, and find the horizontal (x-coordinate) center
        imgSize = img.shape
        xCoord = (imgSize[1] / 2)
        yCoord = (imgSize[0] / 2)
        # Import the dictonary of markers, define the parameters, and load values into vars
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
        # Verify if an ArUco marker is detected
        ids = np.array(ids)
        ids = ids.flatten()
        currID = guidance[index]
        print('ID: ' + str(currID))
        if currID == 0 and music_play:
            music_play = 0
            pygame.mixer.music.play(-1)
        if currID != -1: # If sequence is not finished yet
            if len(corners)> 0: # Verify that it's an Aruco
                #pygame.mixer.music.play()
    #            while pygame.mixer.music.get_busy() == True:
    #                continue
                flag = True # Flag to print to LCD   
                for (markerCorner, markerID) in zip(corners, ids): # Loop through the seen markers
                    if markerID == currID:  # Isolate information for desired marker
                        seemarker = True
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSize, mtx, dist) # Find the rotational and translational                                                                                        
                        # loop over the marker corners
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
                        # Create a marker size constant
                        # Calculate the angle from camera to the object by using the provided formula, 53.76 == FOV of CAMERA          
                        actAngle = round( (61.8/2) * ( (xCoord - cX) / xCoord))
                        z = (tvec[0][0][2]/100)
                        y = (tvec[0][0][0]/100)
                        #markerArea = int ( (topLeft[0] + bottomRight[0]) * (topLeft[1] + bottomRight[1]) )       
                        # Calculate the distance using z-vector of tvec and compensate for angle using trig, divide by 100 to get ft
                        distFromCamToMarker = round( math.sqrt(math.pow(z,2)) * 0.92, 1) + 0.1    # + math.pow(y,2))
                        #distFromCamToMarker = round( z , 1)
                        # Display the ID's and angle
                        actPrint = str(actAngle) + ' degrees'
                        distPrint = str(distFromCamToMarker) + ' ft'
                        #print(distPrint)
                        #cv2.putText(img, distPrint, (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                        cv2.putText(img, actPrint, (topRight[0], topRight[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                        displayLCD(flag, actAngle, distFromCamToMarker) # Send info to Arduino and set sequence flag to go to next marker
                        #print(ids)
                        print(distPrint)
                    else:
                        seemarker = False
        if nextMarker == 1: # Flag to select next marker in sequence
            index += 1  # Move to nextMarker
            nextMarker = 0 # Reset flag
        # show the output image
        cv2.imshow('Image', img)
#    else:
#        cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
cv2.destroyAllWindows()

