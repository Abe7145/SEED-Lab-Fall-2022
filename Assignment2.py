# Abraham Sitanggang
# Mini Project OpenCV Aruco Detection


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy
import sys
import math

# 57 degrees
        
def cv_exercise6():
   # Configure camera to take a video
    cap = cv2.VideoCapture(0)
    while(True):
        #Capture frame-by-frame
        ret, frame = cap.read()
        #Our operations on the frame come here
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Obtain the image pixel size, and find the horizontal (x-coordinate) center
        imgSize = img.shape
        xCoord = (imgSize[1] / 2)
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
                # Use pixel manipulation to determine quadrants, then send the angle corresponding to each quadrant to the arduino
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # Calculate the angle from camera to the object by using the provided formula
                angle = round( (57/2) * ( (cX - xCoord) / xCoord), 2 )
                # Display the ID's and angle 
                anglePrint = str(angle) + ' degrees'
                cv2.putText(img, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                cv2.putText(img, anglePrint, (topRight[0], topRight[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            # show the output image
            cv2.imshow('Image', img)
        #else:
            cv2.imshow('Image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    return 0

cv_exercise6()
