# -*- coding: utf-8 -*-
"""
Code to collect a bunch of images with the webcam for camera calibration
"""
import numpy as np
from playsound import playsound
import glob
import cv2
import time
import os

#getCalibrationImages
#
#PURPOSE: Save a series of images to calculate the camera intrinsics
#
#INPUTS: 
    #numIms:      -Number of images to collect
    #frameWidth:  -Width of individual frame
    #frameHeight: -Height of individual frame
    #dataDir:     -Directory where images should be saved
    #vidIdx:      -Video device number (usually 0)
#OUTPUTS:
    #None
#
#Individual PNG images will be saved to the file path specified
#################################################################
def getCalibrationImages(numIms, frameWidth, frameHeight, dataDir, vidIdx):
    
    timeInterval = 0.5
    
    cap = cv2.VideoCapture(vidIdx)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    
    vid = np.zeros((frameHeight,frameWidth,3,numIms))
    
    # Check success
    if not cap.isOpened():
        raise Exception("Could not open video device")
    for i in range(numIms):
    # Read picture. ret === True on success
        ret, frame = cap.read()
        vid[:,:,:,i] = frame
        playsound('camera-shutter-click-01.wav')
        time.sleep(timeInterval)
    # Close device
    cap.release()
    
    for i in range(numIms):
        fName = os.path.join(dataDir,'calibrationIm_%03d.png'%i)
        cv2.imwrite(fName, vid[:,:,:,i])

#######################################################
#calibrateCamera
#
#PURPOSE: Determine the camera intrinsics
#
#INPUTS: 
    #boardSize:  -How many INTERIOR corners there are in the calibration board
    #squareSize: -How large each square is in real world units
    #calImgDir:  -Directory that holds calibration images
#
#OUTPUTS:
    #mtx   -Camera matrix
    #dist  -Distortion Coefficients
    #rvecs -Rotation vectors for camera intrinsics
    #tvecs -Translation vectors for camera intrinsics
#
#########################################################
def calibrateCamera(boardSize,squareSize,calImgDir):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((np.prod(boardSize),3), np.float32)
    objp[:,:2] = np.mgrid[0:boardSize[0],0:boardSize[1]].T.reshape(-1,2) * squareSize
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    #Get list of png images in the folder
    images = glob.glob(os.path.join(calImgDir,'*.png'))
    imNum = 0
    for fname in images:
        print('working on image %d of %d' % (imNum+1, len(images)))
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        imNum=imNum+1
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, boardSize,None)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
    
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            #img = cv2.drawChessboardCorners(img, boardSize, corners2,ret)
            #cv2.imshow('img',img)
            #cv2.waitKey(500)
        
    #cv2.destroyAllWindows()
    #Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    
    #Calculate reprojection error
    tot_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error

    print("total error: %.3f"% (tot_error/len(objpoints)))
   
    return mtx, dist, rvecs,tvecs

if __name__ == '__main__':
    
    numCalIms = 50
    frameWidth = 1280
    frameHeight = 720
    calDataDir = 'calibration'
    boardSize = (4,3) #Squares
    squareSize = 10 #mm
    
    #######Uncomment to reacquire images
    #getCalibrationImages(numCalIms,frameWidth, frameHeight, calDataDir, 0)
    #############################
    
    #Process images
    mtx, dist, rvecs,tvecs=calibrateCamera(boardSize,squareSize,calDataDir)
    #Save camera intrinsics to an NPZ file format that numpy can unpack
    np.savez('cameraParams', cameraMatrix=mtx, distortionCoefs=dist, rvecs=rvecs, tvecs=tvecs)