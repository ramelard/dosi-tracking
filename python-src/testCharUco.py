# -*- coding: utf-8 -*-
"""
Created on Fri Jul 10 15:07:50 2020

@author: Matthew
"""


import cv2
import numpy as np
from matplotlib import pyplot as plt

	 

# Load the predefined dictionary

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
numRows = 8
numCols = 11
squareLen = 15
markerLen = 10
board = cv2.aruco.CharucoBoard_create(numRows,numCols,squareLen/1000,markerLen/1000,dictionary)
sqSizePx = 200
img = board.draw((sqSizePx*numCols,sqSizePx*numRows))

# Generate the marker
plt.imshow(img)

cv2.imwrite("charucoBoard_8x11_15mmSquare.png", img);

numRows = 4
numCols = 5
squareLen = 15
markerLen = 10
board = cv2.aruco.CharucoBoard_create(numRows,numCols,squareLen/1000,markerLen/1000,dictionary)
sqSizePx = 200
img = board.draw((sqSizePx*numCols,sqSizePx*numRows))

# Generate the marker
plt.imshow(img)

cv2.imwrite("charucoBoard_4x5_15mmSquare.png", img);


#Start capturing images for calibration
cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0
for i in range(300):

    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
except:
    cap.release()

cap.release()
cv2.destroyAllWindows()