# Abraham Sitanggang
# Assignment 2


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy
import sys
import math

def cv_exercise1():
    fileName = input("Please input a file name:")
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
 
    # allow the camera to warmup
    time.sleep(0.5)
 
    # grab an image from the camera
    print("Capturing Image...")
    try:
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        # color correction
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    except:
        print("Failed to capture")
    # save the image to the disk
    print("Saving image: "+fileName)
    # create the proper file location
    fileName = '/home/pi/Desktop/mkdir/'+fileName + '.jpg'
    # Write the picture to location
    cv2.imwrite(fileName, image)
    # Read the picture that was just made
    img = cv2.imread(fileName)
    # Display the picture and wait for user to close window
    #cv2.imshow('Image Display', img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return fileName

#cv_exercise1()

def cv_exercise2():
    # read a picture
    img = cv2.imread('/home/pi/Desktop/mkdir/image1.jpg')
    # Resize the photo by halving x and y pixels
    res = cv2.resize(img, None, fx = 0.5,fy = 0.5, interpolation = cv2.INTER_CUBIC)
    # Display the image and wait for user to close window
    cv2.imshow('Image Display', res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return res

#cv_exercise2()

def cv_exercise3():
   # Read a picture
    img = cv2.imread('/home/pi/Desktop/mkdir/image1.jpg')
    # Convert to gray-scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Display the image and wait for user to close window
    cv2.imshow('Image Display', gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return 0

#cv_exercise3()

def cv_exercise4():
    # Obtain image from first exercise
    imageName = cv_exercise1()
    #print('test')
    imgL = cv2.imread(imageName)
    # Converts image to grayscale
    img = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    #Import the dictonary of markers, define the parameters, and load values into vars
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    # Verify if an ArUco marker is detected
    #print(corners)
    #print(ids)
    if len(corners)> 0:
        # flatten ID list
        ids = ids.flatten()
        # loop over the marker corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract marker corners, convert them to integers, create specific vars for each corner
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
            # draw the ArUco marker ID on the image and display the ID's in output console
            cv2.putText(img, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
        # show the output image
        cv2.imshow("Image with Marker Detection", img)
        cv2.waitKey(0)  
    else:
        print("No markers found.")
    return 0

#cv_exercise4()

def cv_exercise5():
   # Configure camera to take a video
    cap = cv2.VideoCapture(0)
    while(True):
        #Capture frame-by-frame
        ret, frame = cap.read()
        # Convert to grayscale
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            # show the output image
            cv2.imshow('Image', img)
        else:
            cv2.imshow('Image', img)
         # Allow user to close the video feed by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
   # Stop capturing video
    cap.release()
    cv2.destroyAllWindows()
    return 0
    
#cv_exercise5()
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