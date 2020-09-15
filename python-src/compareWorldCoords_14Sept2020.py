# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 09:29:13 2020

@author: Matthew

This script is designed to calculate the world coordinates of the sensor using a few 
different methods. Data were collected on September 14th, 2020
"""
import numpy as np
import cv2
from matplotlib import pyplot as plt
import os
import glob
import time

#generateGridPoints
#
#PURPOSE: Generates evenly spaced points on a grid in OBJECT COORDINATES with 0,0 set as the top left intersection
#
#INPUTS:
#   boardSize -- tuple with the number of interior intersections
#   squareSize -- Physical size of the squares in real units (ie cm, mm, etc.)
#
#OUTPUTS:
#   objp -- object points in mm
#
def generateGridPoints(boardSize, squareSize):
    objp = np.zeros((np.prod(boardSize),3), np.float32)
    objp[:,:2] = np.mgrid[0:boardSize[0],0:boardSize[1]].T.reshape(-1,2) * squareSize
        
    return objp

#getProbeLocation
#
#PURPOSE: Calculate the location of the sensor in world coordinates
#
#INPUTS:
#   cam:       -Dictionary of camera constants (see initCam)
#   flatFrame: -Frame from the camera following flatten
#
#OUTPUTS:
#   P_B:          -Measurement location in world coordinates (mm)
#   rvec:         -Rotation vector of the probe
#   tvec:         -Translation vector of the probe 
#
def getProbeLocation(cam,flatFrame,cropRect,iterFlag):
  
    #Shrink the frame for finding corners
    #flatFrameSmall = cv2.resize(flatFrame,(int(cam["frameSize"][0]*cam["cornerFactor"]),int(cam["frameSize"][1]*cam["cornerFactor"])))
    #Find corners on smaller frame
    
    cropRight = cropRect[0] + cropRect[2]
    cropLeft = cropRect[0]
    cropTop = cropRect[1]
    cropBottom = cropRect[1]+cropRect[3]

    crop_img = flatFrame[cropTop:cropBottom, cropLeft:cropRight]
    patFound,crop_corners=cv2.findChessboardCorners(crop_img,cam["boardShape"])
 
    if patFound:
        corners = crop_corners
        #print(corners)
        #print(corners.shape)
        corners[:,0,0] = crop_corners[:,0,0] + cropLeft
        corners[:,0,1] = crop_corners[:,0,1] + cropTop
        #print(corners)
        #corners2 = corners
        #Do subpixel localization
        #Corner locations are scaled up to correspond to full resolution frame
        corners2 = cv2.cornerSubPix(flatFrame,corners/cam["cornerFactor"],(11,11),(-1,-1),cam["criteria"])
        # noDistMatrix = camDict["cameraMatrix"]
        # noDistMatrix[0,2] = 0
        # noDistMatrix[1,2] = 0
        #Find the pose of the camera based on the corner locations
        #flags = 0 (or cv2.SOLVEPNP_ITERATIVE) is Iterative Levenberg-Marquardt
        #flags = 6 (or cv2.SOLVEPNP_IPPE) is a noniterative solution that depends on all the points being co-planar
        #doc link: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
        #Paper ref: T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation" 
        if iterFlag:
            retVal, rvec,tvec=cv2.solvePnP(cam["boardPts"],corners2,camDict["cameraMatrix"],0,flags=cv2.SOLVEPNP_ITERATIVE)
        else:
            retVal, rvec,tvec=cv2.solvePnP(cam["boardPts"],corners2,camDict["cameraMatrix"],0,flags=cv2.SOLVEPNP_IPPE)
                            
        #Calculate the rotation matrix for this image
        rotMat = cv2.Rodrigues(rvec)[0]
        #Location in world coordinates
        P_A = np.vstack((rotMat.dot(np.array(cam["origin2sensor"]).reshape(3,1)) + tvec.reshape(3,1),1))
     
        return P_A,rvec,tvec,corners
    else:
        return -1,-1,-1,-1
###############################################################################
#updateMap
#
#INPUTS:
#  xPx:         X location of the measurement point (camera space)
#  yPx:         Y location of the measurement point (camera space)
#  dat2map:     The value of the data to map (i.e. mua or chromophore)
#  spreadIm:    The weight image of a single measurement
#  singleMuaIm: A matrix of zeros the size of the camera image. I think it's faster to use this as an input rather than redeclaring each time
#  sumWtIm:     A matrix that sums the weights of all the measurement
#  sumWtMuaIm:  A matrix that's the product of the dat2map and the weight matrix divide by sumWtIm to get weighted average
#OUTPUTS:
#  dat_im:      The current data map
#  singleMuaIm: A matrix of zeros the size of the camera image (see above)
#  sumWtIm:     The updated weight image
#  sumWtMuaIm:  The updated prodcut

#PURPOSE: Returns the current map for display as well as matrices containing the components of the weighted average that need to be kept track of
#         This function should be called every time a new measurement is taken
###############################################################################
def updateMap(xi, yi, thisDat, spreadIm, singleMuaIm, sumWtIm, sumWtMuaIm):
    
    #Get the sizes of all the images    
    spreadImWidth = spreadIm.shape[1]
    spreadImHeight = spreadIm.shape[0]
    imWidth = singleMuaIm.shape[1]
    imHeight = singleMuaIm.shape[0]
  
   
    #st = time.time()
    #If the measurement location is real
    if not np.isnan(xi):
        #Image of zeros to reset singleMuaIm
        zeroSpread = np.zeros(spreadIm.shape)
        #If the spread image goes over the edge you need to crop it by this amt
        cropAmt = [0,0,0,0] #Top, bottom, left, right
       
        #The edges of the spread image in the full image
        leftEdge = int(xi-spreadImWidth/2)
        rightEdge = int(leftEdge+spreadImWidth)
        topEdge = int(yi-spreadImHeight/2)
        bottomEdge = int(topEdge + spreadImHeight)
        #Check if the probe is near the edge
        if leftEdge < 0:
            cropAmt[2] = -leftEdge
            leftEdge = 0
            
        if rightEdge >= imWidth:
            cropAmt[3] = rightEdge - imWidth+1
            rightEdge = imWidth-1
          
        if topEdge < 0:
            cropAmt[0] = -topEdge
            topEdge = 0
          
        if bottomEdge >= imHeight:
            cropAmt[1] = bottomEdge - imHeight+1
            bottomEdge = imHeight-1;
         
        
        #Replace a subset of the single image
        singleMuaIm[topEdge:bottomEdge,leftEdge:rightEdge] = spreadIm[cropAmt[0]:spreadImHeight-cropAmt[1],cropAmt[2]:spreadImWidth-cropAmt[3]]
        
        #Calculate the weighted average (the +=, *= are CRITICAL for speed)
        sumWtIm += singleMuaIm
        singleMuaIm *= thisDat
        sumWtMuaIm += singleMuaIm
        #Reset the single Mua image to all zeros (it's better to store this in the main program so it doesn't have to reallocate memory every frame)
        singleMuaIm[topEdge:bottomEdge,leftEdge:rightEdge] = zeroSpread[cropAmt[0]:spreadImHeight-cropAmt[1],cropAmt[2]:spreadImWidth-cropAmt[3]]
    #Calculate the weighted average, ignore divide by zero errors for this line     
    with np.errstate(invalid='ignore'):       
        dat_im = sumWtMuaIm/sumWtIm
    #Replace NaNs with zero
    dat_im[np.isnan(dat_im)] = 0  

    return dat_im, singleMuaIm, sumWtIm, sumWtMuaIm

###############################################################################
#getColorbarIm
#
#INPUTS:
#   frameWidth:  Width of the final image
#   frameHeight: Height of the image
#   barWidth:    Width of the colorbar
#   cmap:        Colormap to use for the bar
#OUTPUTS:
#   colorBarIm   Image that contains a colorbar on the right side
#
#PURPOSE: Generates a colorbar to be placed in the final image
###############################################################################
def getColorbarIm(frameWidth, frameHeight,barWidth,cmap):
    colorBarIm = np.zeros((frameHeight,frameWidth,3), np.uint8)
 
    #Make the colorbar for the scan. This will stay the same no matter what the range of values is. Only the numbers will change
    #Do this by drawing a rectangle for every value in the color map from 0-255
    for i in range(256):
        thisRect = np.zeros((2,barWidth,3))
        thisColBGR = np.flip(np.array(cmap(i)[0:3]))
        thisRect[:,:,0] = int(thisColBGR[0]*255)
        thisRect[:,:,1] = int(thisColBGR[1]*255)
        thisRect[:,:,2] = int(thisColBGR[2]*255)
        colorBarIm[104+(2*(255-i)):2*(255-i)+106,(frameWidth-barWidth):frameWidth,:] = thisRect
    
    return colorBarIm


if __name__ == '__main__':
    ###CONSTANTS TO CHANGE FOR DIFFERENT DATA###
    IM_HEIGHT = 1024
    IM_WIDTH = 1280
  #  CIRC_DIA_CM = 0.5
    imageDir = '../data/14Sep2020/raw'
    sampName = 'accuracyHoriz_frame'
    ims = glob.glob(os.path.join(imageDir,sampName+'*.png'))
    numIms = len(ims)
    ############################################
    ###Set up camera dictionary for finding location
    camDict = dict()
    intrinsics = np.load('cameraParams.npz')
    ##Some constants taken from the Matlab file
    camDict["cameraMatrix"] = intrinsics['cameraMatrix']
    camDict["origin2sensor"] = (0.0,0.0,0.0)  # (mm) Distance from the top left grid point to the detector
    camDict["squareSize"] = 5
    #Other constants for convenience
    camDict["frameSize"] = (IM_WIDTH,IM_HEIGHT)
    camDict["cornerFactor"] = 1.0
    camDict["displayFactor"] = 1.0
    camDict["boardShape"] = (4,3)
    camDict["criteria"] = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Criteria for subpixel localization of intersection
    #Get all of the grid points in object coordinates
    camDict["boardPts"]=generateGridPoints(camDict["boardShape"],camDict["squareSize"])
    ###################################################################################
    #Hue saturation and value values for segmenting the checkerboard
    lowerHueLow, lowerSatLow, lowerValLow = 35,40,40
    lowerHueHigh, lowerSatHigh, lowerValHigh = 80,255,255
    #Pre-allocate memory for the various images
    #singleMuaIm = np.zeros((IM_HEIGHT,IM_WIDTH))
    #sumWtIm = np.zeros((IM_HEIGHT,IM_WIDTH)) #Weight map (denominator of weighted avg equation)
    #sumWtMuaIm = np.zeros((IM_HEIGHT,IM_WIDTH)) #mua*weight map (numerator of weighted avg equation)
    #Final map to display
    #dat_im = np.zeros((IM_HEIGHT,IM_WIDTH))
    #Weighting for a single image
    #spreadIm = np.loadtxt('./MCSimIm_10mm.txt',delimiter=',')
    #spreadIm = spreadIm/np.max(spreadIm)
    #Im = np.zeros((IM_HEIGHT,IM_WIDTH,3),dtype='uint8')
    
    #flatCol = np.zeros((IM_HEIGHT,IM_WIDTH,3),dtype='uint8')
  
    hsv = np.zeros((IM_HEIGHT,IM_WIDTH,3),dtype='uint8')
    #flatFrame = np.zeros((IM_HEIGHT,IM_WIDTH),dtype='uint8')
    #cmap = plt.cm.jet
    #Pixels per centimeter (this will be updated when an image is read)
    pxPerCm = 5
    #Display a circle at the measurement location of this diameter (in pixels)
    #circDiaPx = CIRC_DIA_CM * pxPerCm
    rotations = []
    translations = []
    locX = []
    locY=[]
    locZ=[]
    
    locXit = []
    locYit = []
    locZit = []
    cv2.namedWindow('stopMotion')
    for j in range(2):
        st=time.time()
        for i in range(numIms):
            thisIm = cv2.imread(ims[i])
            grayIm = cv2.cvtColor(thisIm,cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(thisIm, cv2.COLOR_BGR2HSV)
            #Segment the image
            maskLower = cv2.inRange(hsv,(lowerHueLow, lowerSatLow, lowerValLow),(lowerHueHigh, lowerSatHigh,lowerValHigh))
            contoursLower,_ = cv2.findContours(maskLower, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contoursLower) != 0:
                mlower = max(contoursLower, key=cv2.contourArea)
                cropX,cropY,cropW,cropH = cv2.boundingRect(mlower)
                if cropW*cropH > 10000: #Set minimum size for the green rectangle around checkerboard for it to "count"
                    ploc3d, rvec,tvec,corners=getProbeLocation(camDict,grayIm,(cropX,cropY,cropW,cropH),j)
                    if j == 0:
                        #If the probe location is unknown ploc3d returns (-1,-1,-1)
                        rotations.append(rvec)
                        translations.append(tvec)
                        locX.append(ploc3d[0][0])
                        locY.append(ploc3d[1][0])
                        locZ.append(ploc3d[2][0])
                    else:
                        #If the probe location is unknown ploc3d returns (-1,-1,-1)
                        #rotations.append(rvec)
                        #translations.append(tvec)
                        locXit.append(ploc3d[0][0])
                        locYit.append(ploc3d[1][0])
                        locZit.append(ploc3d[2][0])
                    cv2.drawChessboardCorners(thisIm,(4,3),corners,1)
                    cv2.imshow('stopMotion',thisIm)
                    cv2.waitKey(10)
        el = time.time()-st
        print(el)
    cv2.destroyAllWindows()
      # Creating figure 
    fig = plt.figure(figsize = (10, 7)) 
    ax = plt.axes() 
    # Creating plot 
    ax.plot(locX, locY,'o');
    ax.plot(locXit,locYit,'o')
    
    fig = plt.figure()
    ax2 = plt.axes()
    ax2.plot(locZ,'o')
    ax2.plot(locZit,'o')
    
    rotVec1 = (rotations[0][0][0],rotations[0][1][0],rotations[0][2][0])
    rotMat1 = cv2.Rodrigues(rotVec1)[0]
    tVec1 = [translations[0][0][0],translations[0][1][0],translations[0][2][0],1]
    M = np.zeros((4,4))
    M[0:3,0:3] = rotMat1
    M[:,3] = tVec1
    Minv = np.linalg.pinv(M)
    worldX = []
    worldY = []
    worldZ = []
    for i in range(len(locX)):
        thisLoc = np.dot(Minv,np.vstack((locX[i],locY[i],locZ[i],1)))
        worldX.append(thisLoc[0])
        worldY.append(thisLoc[1])
        worldZ.append(thisLoc[2])
    
    #Creating figure 
    fig = plt.figure(figsize = (10, 7)) 
    ax = plt.axes(projection='3d') 
    # Creating plot 
    ax.scatter3D(worldX,worldY,worldZ,'o');
    plt.ion()
    
    
            
