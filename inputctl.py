import cv2
import numpy as np
from Queue import Queue
import time

def inpctl(inpQ):
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

		starttime = time.time()

		# Capturing Image
		retL = camL.grab()
		retR = camR.grab()

		print "Input Control",time.time()-starttime

		# Retrieving Image
		retL, imgl = camL.retrieve()
		retR, imgr = camR.retrieve()

		camL.release()
		camR.release()

		# Convert to Grayscale
		imgL = cv2.cvtColor(imgl,cv2.COLOR_BGR2GRAY)
		imgR = cv2.cvtColor(imgr,cv2.COLOR_BGR2GRAY)

		# Parsing input to Processing Module through Queue
		inpT= (imgL, imgR)
		inpQ.put(inpT)
