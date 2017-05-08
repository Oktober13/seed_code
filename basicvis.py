""" Experiment with face detection and image filtering using OpenCV """

import cv2
import numpy as np

cap = cv2.VideoCapture(1)
temp = True
flag = 0 #Number of current frames- Keeping track of runtime
maxruntime = 100 #frames- Max runtime hard cutoff

while(temp == True):
	#Capture frame-by-frame
	ret, frame = cap.read()
	if not ret:
		print ret
		print frame
		temp == False

	#Display the resulting frame

	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	if flag > maxruntime:
		temp = cap.release() #if frame read correctly == True
		cv2.destroyAllWindows()

	flag = flag + 1

#release the capture
temp = cap.release() #if frame read correctly == True
cv2.destroyAllWindows()