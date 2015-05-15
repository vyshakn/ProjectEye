
#           Incorrect Code

import numpy as np
import cv2
from time import sleep

print "Welcome\n"

numBoards = 50  #how many boards would you like to find

board_sz = (7,4)

square_sz = 2.9

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all the images.
object_points = [] # 3d point in real world space
imagePoints1 = [] # 2d points in image plane.
imagePoints2 = [] # 2d points in image plane.

corners1 = []
corners2 = []

obj = np.zeros( (np.prod(board_sz), 3), np.float32 )
obj[:,:2] = np.indices(board_sz).T.reshape(-1, 2)
obj *= square_sz


vidStreamL = cv2.VideoCapture()  # index of your camera
vidStreamR = cv2.VideoCapture()  # index of your camera
success = 0
k = 0
found1 = False
found2 = False

vidStreamL.open(1)
vidStreamL.read()
x = raw_input("Is Left? (y/n)")
if x == 'y' :
    LT = 1
    RT = 2
else :
    LT = 2
    RT = 1
vidStreamL.release()

sleep(3)

while (success < numBoards):
    vidStreamL.open(LT)
    vidStreamL.set(3,320)
    vidStreamL.set(4,240)
    vidStreamR.open(RT)
    vidStreamR.set(3,320)
    vidStreamR.set(4,240)
    sleep(2)
    print "Capturing image in 3 .. "
    sleep(1)
    print "2 .. "
    sleep(1)
    print "1 .. "
    sleep(1)
    retL = vidStreamL.grab()
    retR = vidStreamR.grab()
    retL, img1 = vidStreamL.retrieve()
    retR, img2 = vidStreamR.retrieve()
    vidStreamL.release()
    vidStreamR.release()
    print "Ok"
    height, width, depth  = img1.shape
    gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

    found1, corners1 = cv2.findChessboardCorners(gray1, board_sz)
    found2, corners2 = cv2.findChessboardCorners(gray2, board_sz)

    if (found1):
        cv2.cornerSubPix(gray1, corners1, (5, 5), (-1, -1),criteria)
        cv2.drawChessboardCorners(gray1, board_sz, corners1, found1)

    if (found2):
        cv2.cornerSubPix(gray2, corners2, (5, 5), (-1, -1), criteria)
        cv2.drawChessboardCorners(gray2, board_sz, corners2, found2)

    cv2.imshow('image1', gray1)
    cv2.imshow('image2', gray2)
    k = cv2.waitKey(10) & 0xFF
    if found1 !=0 and found2 != 0 :
        print "\nFound!"

    if (k == ord('q')):
        break
    if (found1 != 0 and found2 != 0): # k == ord('y') and
        imagePoints1.append(corners1.reshape(-1, 2))
        imagePoints2.append(corners2.reshape(-1, 2))
        object_points.append(obj)
        print "Corners stored %d\n" % success
        success += 1
        if (success >= numBoards):
            break

cv2.destroyAllWindows()
vidStreamL.open(LT)
vidStreamL.set(3,320)
vidStreamL.set(4,240)
vidStreamR.open(RT)
vidStreamR.set(3,320)
vidStreamR.set(4,240)

print "Starting Calibration\n"


retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(object_points, imagePoints1, imagePoints2, (width, height))
print "Done Calibration\n"
print "Starting Rectification\n"
R1 = np.zeros(shape=(3,3))
R2 = np.zeros(shape=(3,3))
P1 = np.zeros(shape=(3,4))
P2 = np.zeros(shape=(3,4))
Q = np.zeros(shape=(4,4))

#(roi1, roi2) = cv2.cv.StereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,(width, height), R, T, R1, R2, P1, P2, Q=None, flags=cv2.cv.CV_CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(0, 0))
cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,(width, height), R, T, R1, R2, P1, P2, Q, flags=cv2.cv.CV_CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(0,0))
#stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,(width, height), R, T)

print "Done Rectification\n"
print "Applying Undistort\n"

map1x, map1y = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (width, height), cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (width, height), cv2.CV_32FC1)

print "Undistort complete\n"

while(True):
    retL, img1 = vidStreamL.read()
    retR, img2 = vidStreamR.read()
    imgU1 = np.zeros((height,width,3), np.uint8)
    imgU2 = np.zeros((height,width,3), np.uint8)
    imgU1 = cv2.remap(img1, map1x, map1y, cv2.INTER_LINEAR) #, cv2.BORDER_CONSTANT, 0)
    imgU2 = cv2.remap(img2, map2x, map2y, cv2.INTER_LINEAR) #, cv2.BORDER_CONSTANT, 0)
    cv2.imshow("imageL", img1)
    cv2.imshow("imageR", img2)
    cv2.imshow("image1L", imgU1)
    cv2.imshow("image2R", imgU2)
    k = cv2.waitKey(5) & 0xFF;
    if(k==ord('q')):
        break
