import cv2
import numpy as np
from Queue import Queue
from multiprocessing.pool import ThreadPool
import matcher
import stereo
import tts
import time

def process(inpQ) :
    while(True):
        inpt = 0

        # Getting input images from Input Control Module from Queue
        while not inpQ.empty():
            inpt = inpQ.get()
            inpQ.task_done()

        if inpt:
            starttime = time.time()
            imgL = inpt[0]
            imgR = inpt[1]

            pool1 = ThreadPool(processes=1)
            pool2 = ThreadPool(processes=1)

            # Calling Feature matching function as seperate thread
            matching = pool1.apply_async(matcher.match, (imgL,))

            # Calling Stereo Matching function as seperate thread
            stereos = pool2.apply_async(stereo.stereo, (imgL,imgR))

            point = 0
            disparity = np.float32([])

            # Waiting for values from threads
            while(True) :
                point = matching.get()
                disparity = stereos.get()
                if point and disparity.size:
                    break
                else :
                    point = 0
                    disparity = np.float32([])

            output = 0

            if point[0] > 0 :
                if disparity[point[1],point[0]] > 0.3 :
                    output = "Book is Near"

            print "Processing Module",time.time()-starttime

            # Calling text to speech Module
            if output:
                tts.tts(output)
