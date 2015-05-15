import numpy as np
import cv2
from time import sleep

def match(img) :

    img2 = cv2.imread('obj.png',0) # trainImage

    orb = cv2.ORB()
    kp2, des2 = orb.detectAndCompute(img2,None)

    bf = cv2.BFMatcher()

    kp1, des1 = orb.detectAndCompute(img,None)
    matches = bf.knnMatch(des1,des2, k=2)
    #print matches
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.6*n.distance:
                good.append(m)

    keypq = []
    if len(good) > 0 :
        keypq.append(kp1[good[0].queryIdx])

    if len(keypq):
        return keypq[0].pt
    else :
        return (-1 , -1)
