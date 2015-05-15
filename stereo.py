
import numpy as np
import cv2
import time
def stereo(imgL,imgR):

	window_size = 3
	min_disp = 16
	num_disp = 112-min_disp
	stereo = cv2.StereoSGBM(minDisparity = min_disp,
	numDisparities = num_disp,
	SADWindowSize = window_size,
	uniquenessRatio = 2,
	speckleWindowSize = 75,
	speckleRange = 16,
	disp12MaxDiff = 1,
	P1 = 8*3*window_size**2,
	P2 = 32*3*window_size**2,
	fullDP = False
	)

	disp= stereo.compute(imgL,imgR).astype(np.float32) / 16.0

	return (disp-min_disp)/num_disp
