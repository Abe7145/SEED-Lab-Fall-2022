from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import glob
     
#camera = PiCamera()
#rawCapture = PiRGBArray(camera)
#time.sleep(0.5)
#try:
#    camera.capture(rawCapture, format = "bgr")
#    image = rawCapture.array
#except:
#    print('Failed to capture')
#    
#cv2.imwrite('chessboard.jpg', image)
     
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
    
    

img = cv2.imread('calib_radial.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
   
# If found, add object points, image points (after refining them)
if ret == True:
    objpoints.append(objp)
   
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    imgpoints.append(corners2)
   
    # Draw and display the corners
    cv2.drawChessboardCorners(img, (7,6), corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey(1000)
else:
    print('Fail')

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

img1 = cv2.imread('calib_radial.jpg')
h, w = img1.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


dst = cv2.undistort(img1, mtx, dist, None, newcameramtx) 
 # crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)

np.savez('calibrationvalues', old = mtx, new = newcameramtx, dist = dist)

#mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
#dst = cv2.remap(img1,mapx,mapy,cv2.INTER_LINEAR)
## crop the image
#x,y,w,h = roi
#dst = dst[y:y+h, x:x+w]
#cv2.imwrite('calibresult.png',dst)

print(mtx)
print('gamer')
print(newcameramtx)
print('gamer2')
print(dst)



