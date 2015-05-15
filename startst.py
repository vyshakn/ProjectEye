import cv2
import numpy as np
from Queue import Queue
import time
import matcher
import stereo
import tts

camL = cv2.VideoCapture() # Left camera
camR = cv2.VideoCapture() # Right Camera

camL.open(1)
camL.read()
x = raw_input("Is Left? (y/n)")
if x == 'y' :
    L = 1
    R = 2
else :
    L = 2
    R = 1
camL.release()

while(True):
    camL.open(L)
    camL.set(3,1024)
    camL.set(4,768)
    camR.open(R)
    camR.set(3,1024)
    camR.set(4,768)

    # Timelapse delay between input images
    time.sleep(3)

    starttimei = time.time()

    # Capturing Image
    """
    retL = camL.grab()
    retR = camR.grab()
    """
    retL, imgl = camL.read()
    retR, imgr = camR.read()

    print "Input Control",time.time()-starttimei
    """
    # Retrieving Image
    retL, imgl = camL.retrieve()
    retR, imgr = camR.retrieve()
    """
    camL.release()
    camR.release()

    # Convert to Grayscale
    imgL = cv2.cvtColor(imgl,cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(imgr,cv2.COLOR_BGR2GRAY)

    # Processing

    starttimep = time.time()

    # Calling Feature matching function as seperate thread
    point = matcher.match(imgL)
    x = time.time()
    print "Match", x - starttimep
    # Calling Stereo Matching function as seperate thread
    disparity = stereo.stereo(imgL,imgR)
    print "Stereo " ,time.time() - x
    output = 0

    if point[0] > 0 :
        if disparity[point[1],point[0]] > 0.2 :
            output = "Object is Near"
        else :
            output = "Object is Far"
    print "Processing Module",time.time()-starttimep
    # Calling text to speech Module
    if output:
        tts.tts(output)
