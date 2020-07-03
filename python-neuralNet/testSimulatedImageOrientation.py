# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 11:45:06 2020
Script to test localization of training images
@author: Matthew
"""
import cv2
import numpy as np
import glob
import os
import csv
from matplotlib import pyplot as plt

def getCheckerRect(corners):
    left = int(np.round(np.min(corners[:,0,0])))
    top = int(np.round(np.min(corners[:,0,1])))
    width = int(np.round(np.max(corners[:,0,0]) - left))
    height = int(np.round(np.max(corners[:,0,1])- top))
    
    return [left, top, width, height]

if __name__ == '__main__':
    dataDir = 'D:\\Work\\RoblyerLab\\trackDOSI\\plots\\simProbe\\'
    
    boardShape = (7,6)
    squareSize = 6.7
    objp = np.zeros((np.prod(boardShape),3), np.float32)
    objp[:,:2] = np.mgrid[0:boardShape[0],0:boardShape[1]].T.reshape(-1,2) * squareSize
    frameWidth = 2048 #(px) Width of video frame
    frameHeight = 2048 #(px) Height of video frame
    findCornersFactor = .5 #Finding checkerboard corners is slow, so scale down the image by this factor prior to looking for them
    displayFactor = .5 #The images are too big to fit on my screen, so scale them down by this much before showing
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Criteria for subpixel localization of intersections
    cameraMatrix = np.eye(3)
    
    f=glob.glob(os.path.join(dataDir,'*.tif')) #list of avi files
    #Set the board rectangle initially empty. This holds the coordinates of the chessboard !!IN flatFrameSmall COORDINATES!!
    boardRect = []
    firstTime = True
    sensorLoc = np.array([0,0,0])
    sensorMask = np.ones(len(f))
    sensorPathWorld = np.zeros((len(f),3)) #Path of sensor in World coordinates relative to upper left intersection on frame 1
    sensorPathImage = np.zeros((len(f),2)) #Path of detector in Camera coordinates
    transVecs = np.zeros((len(f),3))
    frameNum = 0
    for fname in f:
        cap = cv2.VideoCapture(fname)
        ret,flatFrame = cap.read()
        flatFrame = flatFrame[:,:,0]
        #Shrink the frame for finding corners
        flatFrameSmall = cv2.resize(flatFrame,(int(frameWidth*findCornersFactor),int(frameHeight*findCornersFactor)))
        
       
        patFound,corners=cv2.findChessboardCorners(flatFrameSmall,boardShape)
        
        
        #If a checkerboard is recognized
        if patFound:
            boardRect = getCheckerRect(corners)
            #Do subpixel localization
            #Corner locations are scaled up to correspond to full resolution frame
            corners2 = cv2.cornerSubPix(flatFrame,corners/findCornersFactor,(11,11),(-1,-1),criteria)
            
            #Find the pose of the camera based on the corner locations
            #flags = 0 (or cv2.SOLVEPNP_ITERATIVE) is Iterative Levenberg-Marquardt
            #flags = 6 (or cv2.SOLVEPNP_IPPE) is a noniterative solution that depends on all the points being co-planar
            #doc link: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
            #Paper ref: T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation" 
            retVal, rvec,tvec=cv2.solvePnP(objp,corners2,cameraMatrix,0,flags=cv2.SOLVEPNP_IPPE)
            
            if firstTime: #If this is the first frame do some calculations
                firstTime = False #make sure this only runs once
                firstIm = flatFrame #Save the first image because why not
                rvec0 = rvec #Save the rotation vector
                rotMat0 = cv2.Rodrigues(rvec0)[0] #Turn rotation vector into matrix
                #Make matrix 4x4
                rotMat4x4 = np.vstack((np.hstack((rotMat0,np.array([0,0,0]).reshape(3,1))),np.array([0,0,0,1]).reshape(1,4)))
                tvec0 = tvec #Save translation vector
                #Make translation matrix 4x4
                tMat4x4 = np.vstack((np.hstack((np.zeros((3,3)),tvec.reshape(3,1))),np.array([0,0,0,0]).reshape(1,4)))
                #Combine both matricies
                #M0 is the transformation from World space to Camera Space
                M0 = rotMat4x4 + tMat4x4
                #Invert M0 which is now the transformation from Camera Space to World Space
                M0_inv = np.linalg.pinv(M0)
                
                
            #Calculate the rotation matrix for this image
            rotMat = cv2.Rodrigues(rvec)[0]
            #Location in world coordinates relative to origin of this image
            P_A = np.vstack((rotMat.dot(sensorLoc.reshape(3,1)) + tvec.reshape(3,1),1))
            #Location in world coordinates relative to origin of first image
            P_B = M0_inv.dot(P_A)
            
            #Save the sensor path in world coordinates
            sensorPathWorld[frameNum,:]=P_B[0:3].reshape(3)
            transVecs[frameNum,:] = tvec.flatten()
            #Project the detector point in world coordinates into Camera Space for visualization
            imgpts, _ = cv2.projectPoints(P_B[0:3], rvec0, tvec0, cameraMatrix, 0)
            # imgp1, _ = cv2.projectPoints(p1, rvec, tvec, cameraMatrix, 0)
            # imgp2, _ = cv2.projectPoints(p2, rvec, tvec, cameraMatrix, 0)
            # imgp3, _ = cv2.projectPoints(p3, rvec, tvec, cameraMatrix, 0)
            # imgp4, _ = cv2.projectPoints(p4, rvec, tvec, cameraMatrix, 0)
            ptXInt = int(np.floor(imgpts[0,0,0]*displayFactor)) #Cast to integer so it can be drawn
            ptYInt = int(np.floor(imgpts[0,0,1]*displayFactor))
            #Save sensor path in the image for visuilization
            sensorPathImage[frameNum,:]=[ptXInt,ptYInt]
            #Rescale image and convert to color for display
            frameCol = cv2.cvtColor(cv2.resize(flatFrame,(int(frameWidth*displayFactor),int(frameHeight*displayFactor))),cv2.COLOR_GRAY2BGR)
            #Plot grid corners as circles
            #for i in range(len(corners2)):
            #cv2.circle(frameCol,(int(np.floor(corners2[0,0,0]*displayFactor)),int(np.floor(corners2[0,0,1]*displayFactor))),10,(0,0,255),2)
            cv2.drawChessboardCorners(frameCol,(4,3),corners2,patFound)
           # cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*displayFactor)),int(np.floor(imgp1[0,0,1]*displayFactor))),
            #         (int(np.floor(imgp2[0,0,0]*displayFactor)),int(np.floor(imgp2[0,0,1]*displayFactor))),(0,255,0),3)
            #cv2.line(frameCol,(int(np.floor(imgp2[0,0,0]*displayFactor)),int(np.floor(imgp2[0,0,1]*displayFactor))),
            #         (int(np.floor(imgp3[0,0,0]*displayFactor)),int(np.floor(imgp3[0,0,1]*displayFactor))),(0,255,0),3)
            #cv2.line(frameCol,(int(np.floor(imgp3[0,0,0]*displayFactor)),int(np.floor(imgp3[0,0,1]*displayFactor))),
            #         (int(np.floor(imgp4[0,0,0]*displayFactor)),int(np.floor(imgp4[0,0,1]*displayFactor))),(0,255,0),3)
          
            #Draw circle at detector location
            cv2.circle(frameCol,(ptXInt,ptYInt),10,(255,0,0),-1)
        #If the grid pattern isn't found, do nothing and mark the sensor mask
        else:
            #frameCol = np.zeros((int(frameWidth*displayFactor),int(frameHeight*displayFactor)),dtype='uint8')
            sensorMask[frameNum] = False
            boardRect = []
        
        #Show the frame
        # try:
        #     cv2.imshow('frame',frameCol)
        #     #Press q to quit out of the loop
        #     if cv2.waitKey(100) & 0xFF == ord('q'):
        #         break
        # except:
        #     continue
        frameNum = frameNum+1
        cap.release()
    
    cv2.destroyAllWindows()
    
    dataFile = 'C:\\Users\\Matthew\\Documents\\GitHub\\dosi-tracking\\python-neuralNet\\probePosition.csv'
    numIms = len(sensorMask)
    data = np.zeros((690,6))
    with open(dataFile,newline='') as f:
        reader = csv.reader(f)
        line_ct = 0
        for row in reader:
            if line_ct != 0:
                data[line_ct-1,:] = row
            line_ct = line_ct+1
    
    plt.plot(data[:,1],sensorPathWorld[310::,1],'o')
    plt.show()
