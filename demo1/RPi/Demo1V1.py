# SEEDsters
# Demo 1 CV Code

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import math
import smbus
import smbus2
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
#Python code for pi-
#take the setpoint that the pi reads from the aruco marker and send it to arduino

#lcd_columns = 16
#lcd_rows = 2
#
#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
#
#abus = smbus.SMBus(1)
#address = 0x2a
#slave_address = 0x00
#angle = 0
#
#data = 0
#currentPos = 2

#def writeNumber(variableName):
#    
#    #bus.write_byte(address,variableName)
#    data = ""
#    for i in range(0, 5):
#            data += chr(abus.read_byte(address));
#    print (data)
#    time.sleep(1);
#    
#    lcd.message = data
#    
#    return -1

def writeNumber(variableName):
    try:
        abus.write_byte(address,variableName)
    except:
        print('io error')
    
    try:
        data = ""
        for i in range(0,3):
            data += chr(abus.read_byte(address));
        #print(data)
        #print('\n')
    except:
        print('io error')
    
    tpos = ''
    if ( variableName == 0 ):
        tpos = '00'
    elif ( variableName == 1 ):
        tpos = '090'
    elif ( variableName == 2 ):
        tpos = '180'
    elif ( variableName == 3 ):
        tpos = '270'
    lcd.message = 'Cpos: ' + str(data) + '\nTpos: ' + str(tpos) 
    
    return -1

def displayLCD(flag, angle):
    if flag == True:
        lcd.message = 'Marker found! \nAngle: ' + str(angle)
    else:
        lcd.message = 'No marker \nfound.'
    return -1

    
def readLCD(setpoint):
    #posData = abus.read_byte_data(slave_address, address)
    #posPrint = '\nPosition: ' + str(posData)
    #currentPos = readNumber()
    #print(currentPos)
    setPrint = 'Setpoint: ' + str(setpoint)
    lcd.message = setPrint #+ posPrint

# Configure camera to take a video
# Video capture settings
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 3);
cap.set(cv2.CAP_PROP_FPS, 30);
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

npzfile = np.load('calibrationvalues.npz')

while(True):
    #Capture frame-by-frame
    ret, frame = cap.read()
    #Our operations on the frame come here
    #resize = cv2.resize(frame, (600, 600))
    #Distortion stuff supposdely
    img = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(npzfile['old'], npzfile['dist'], (w,h), 1, (w,h))
    img = cv2.undistort(img, npzfile['old'], npzfile['dist'], None, npzfile['new']) 
    # crop the image
    x,y,w,h = roi
    img = img[y:y+h, x:x+w]
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
        # flatten ID list
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
            # Calculate the angle from camera to the object by using the provided formula
            actAngle = round( (57/2) * ( (xCoord - cX) / xCoord), 2 )
            # Radians
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
cap.release()
cv2.destroyAllWindows()
