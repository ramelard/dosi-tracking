# -*- coding: utf-8 -*-
"""
Created on Fri May 29 15:35:26 2020

@author: Matthew
"""

import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import cv2
import random
import csv
import pickle
import os

#createChessboard
#
#PURPOSE: Genreate a chessboard pattern for making simulated data
#
#INPUTS:
#   boardShape:   Tuple with the number of internal intersections in the x- and y-directions
#   squareSizePx: Size of each board square in pixels
#OUTPUTS:
#    chess:       3D matrix with dimensions[xpx, ypx, [x,y,z,color]]
#
#Notes: the X and Y values are centered at 0,0 and the color is 0 or 255
def createChessboard(boardShape, squareSize_mm):
    pxPerSquare = 40
    blackLine= np.zeros((1,pxPerSquare))       #A single black edge
    whiteLine = np.ones((1,pxPerSquare))*255   #A single white edge
    #Start with empty arrays for each line
    blackFirstLine = np.array([]) #Line starting with black square
    whiteFirstLine = np.array([]) #Line starting with white square
    #Preallocate for the full board
    chess = np.zeros(((boardShape[1]+1)*pxPerSquare,(boardShape[0]+1)*pxPerSquare,4))
    #Calculate number of pixels needed in both axes
    numPxX = (boardShape[0]+1) * pxPerSquare
    numPxY = (boardShape[1]+1) * pxPerSquare
    
    mmX = squareSize_mm * (boardShape[0] + 1)
    mmY = squareSize_mm * (boardShape[1] + 1)
    
    #Values for the X and Y axis centered at (0,0)
    xVals= np.linspace(-mmX/2, mmX/2,numPxX)
    yVals = np.linspace(-mmY/2, mmY/2,numPxY)
    #Meshgrid the arrays to get grid of X,Y locations        
    xv,yv = np.meshgrid(xVals,yVals)
    zv = np.zeros(xv.shape) #Z is always 0 when starting out
    #Assign X,Y,and Z coordinates
    chess[:,:,0] = xv
    chess[:,:,1] = yv
    chess[:,:,2] = zv
    #Generate lines from edges along the Y axis
    for i in range(boardShape[1]+1):
        if i % 2 == 0:
            blackFirstLine = np.append(blackFirstLine,blackLine)
            whiteFirstLine = np.append(whiteFirstLine,whiteLine)
        else:
            blackFirstLine = np.append(blackFirstLine,whiteLine)
            whiteFirstLine = np.append(whiteFirstLine,blackLine)
    #Stack lines into 2D image
    bfFlag = False #Flag for whether this should be a black square first or white square first row
    for j in range((boardShape[0]+1)*pxPerSquare):
        #Flip the rows after one square complete
        if j % pxPerSquare == 0:
            bfFlag = not bfFlag
        #Append a line depending on whether it should be black or white first
        if bfFlag:
            chess[:,j,3] = blackFirstLine
        else:
            chess[:,j,3] = whiteFirstLine
            
    return chess
    
#transformPts
#
#PURPOSE: Transforms the points of the chessboard found using the generateChessboard function
#         by rotating around the X, Y, and Z axis and then translating in all 3 axes
#
#INPUTS:
#   objPoints:   HxW matrix containing the X, Y, and Z coordinates to be transformed
#   rotation:    1x3 vector with rotations around the X, Y, and Z axes in radians
#   translation: 1x3 vector with translation in X,Y, and Z in units of Pixels
def transformPts(objPoints,rotation,translation):
    #Calculate the rotation matrices
    rotXMatrix = np.array([[1,0,0],[0,np.cos(rotation[0]),-np.sin(rotation[0])],[0,np.sin(rotation[0]),np.cos(rotation[0])]])
    rotYMatrix = np.array([[np.cos(rotation[1]),0,np.sin(rotation[1])],[0,1,0],[-np.sin(rotation[1]),0,np.cos(rotation[1])]])
    rotZMatrix = np.array([[np.cos(rotation[2]),-np.sin(rotation[2]),0],[np.sin(rotation[2]),np.cos(rotation[2]),0],[0,0,1]])
    #Calculate the total rotation around all 3 axes
    totalRotMatrix = rotZMatrix @ rotYMatrix @ rotXMatrix
    #Allocate memory to hold transformed image
    transPts = np.zeros(objPoints.shape)
    #Copy the color value from objPoints to new matrix
    transPts[:,:,-1] = objPoints[:,:,-1]
    
    #Transform each point
    for i in range(objPoints.shape[0]):
        for j in range(objPoints.shape[1]):
            thisPt = objPoints[i,j,0:3] #Get the point
            #I could probably combine these into 1 4D matrix if I really wanted to, right?
            rotPt = totalRotMatrix.dot(thisPt) #Rotate the point
            transPt = np.array([rotPt[0]+translation[0],rotPt[1]+translation[1],rotPt[2]+translation[2]]) #Translate the point
            transPts[i,j,0:3] = transPt #Assign the new point
            
    return transPts

#projectPts
#
#PURPOSE: Projects objPoints onto a screen imgDist away from camera pinhole. Makes parallel lines approach a vanishing point
#
#INPUTS:
#   objPoints: X,Y, and Z points of the object
#   imFOV: FOV of the camera
#
#Notes: I'm not sure if this is correct. I got the formula from: https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
#       Unfortunately, there's no reference. I'm very unsure about the sx and sy which is stated to be "the display size"
#       I don't know what "display" they are referring to. I'm using the FOV of the image which doesn't seem crazy. The images
#       generated look approximately right to my eye given the distances involved, so I'm going with it. This will probably need to be
#       changed when working with real images
#def projectPts(objPoints,imgDist):
def projectPts(objPoints, rvec, tvec):
    ##Some constants taken from the Matlab file
    cameraMatrix = np.matrix([[2858.2, 0     , 1009.3],\
                              [0     , 2849.7, 1135.8],\
                              [0     ,0      ,    1  ]]) #Camera intrinsics
    #distCoefs = np.array([-.2499,.2871,0,0]) #Distortion coefficients
    #rx = 36 #mm. Size of the recording medium (using 35mm film size)
    #ry = 24 #mm
    #rz = -50 #mm Distance of the recording medium from camera pinhole (using focal length of lens)
    rMat = cv2.Rodrigues(rvec)[0]
    rotMat4x4 = np.vstack((np.hstack((rMat,np.array([0,0,0]).reshape(3,1))),np.array([0,0,0,1]).reshape(1,4)))
    #Make translation matrix 4x4
    tMat4x4 = np.vstack((np.hstack((np.zeros((3,3)),tvec.reshape(3,1))),np.array([0,0,0,0]).reshape(1,4)))
    #Combine both matricies
    #M0 is the transformation from World space to Camera Space
    extrinsicMatrix = rotMat4x4 + tMat4x4
    #Allocate memory for new image
    projPts = np.zeros((objPoints.shape[0],objPoints.shape[1],3))
    #Copy color value into the last page
    projPts[:,:,-1] = objPoints[:,:,-1]
    #Project the points onto the screen
    for i in range(objPoints.shape[0]):
        for j in range(objPoints.shape[1]):
            thisPt = objPoints[i,j,0:3]
            cameraCoords = extrinsicMatrix.dot(np.append(thisPt,1))
            #thisX = (thisPt[0]*imFOV[0])/(thisPt[2]*rx) * rz
            #thisY = (thisPt[1]*imFOV[1])/(thisPt[2]*ry) * rz
            #projPts[i,j,0:2] = np.array([imgDist/thisPt[2]*thisPt[0],imgDist/thisPt[2]*thisPt[1]])
            pPoint = cameraMatrix.dot(cameraCoords[0:3])
            projPts[i,j,0:2] = np.array([pPoint[0,0]/pPoint[0,2],pPoint[0,1]/pPoint[0,2]])
            
    # if rz < 0:
    #     projPts = np.flipud(np.fliplr(projPts))
        
    return projPts

#getSimulatedImage
#
#PURPOSE: Generate a simulated image from the given matrix
#
#INPUTS:
#   imSzPx:         Size of the image in pixels (HxW)
#   imFOV:          Camera field of view in mm
#   projectedImage: X,Y location of the projection (in mm) and the color at each location
#   pxPermm:        Pixel size IN CHESSBOARD COORDINATES
#
#NOTES: I'm not sure if this is 100 percent right. See notes on "boolX" and "boolY" lines
def getSimulatedImage(imSzPx, projectedImage):
    
    bgVal = 128 #Background value to use when the target isn't in frame
    #Allocate memory for simulated image
    simImg = np.ones((imSzPx[1],imSzPx[0]),'uint8') * bgVal
    #X and Y location of each pixel in mm
    #pxLocX = np.linspace(-imFOV[0]/2,imFOV[0]/2,imSzPx[0])
    #pxLocY = np.linspace(-imFOV[1]/2, imFOV[1]/2,imSzPx[1])
    #Size of each pixel (mm)
    #pxSz = pxLocX[1]-pxLocX[0]
    #Bounding box for the probe
    minLocX = np.min(p[:,:,0])
    minLocY = np.min(p[:,:,1])
    maxLocX = np.max(p[:,:,0])
    maxLocY = np.max(p[:,:,1])
    
    #mmPerPx = 1/pxPermm
    
    #Iterate through each pixel in the final simulated image
    for x in range(int(np.floor(minLocX)),int(np.ceil(maxLocX)+1)):
        for  y in range(int(np.floor(minLocY)),int(np.ceil(maxLocY)+1)):
            #The edges of this pixel
             #locMM_x = [pxLocX[x],pxLocX[x]+pxSz]
             #locMM_y = [pxLocY[y],pxLocY[y]+pxSz]
             #If the pixel is outside of the probe bounding box set it to background value
             if x < 0 or y < 0:
                 continue
             elif x >= imSzPx[1] or y >= imSzPx[0]:
                 continue
             #Pixel is inside the bounding box    
             else:
                #Find the projected image pixels that are inside of this pixel
                #The chessboard pixel location is assumed to be the center of the pixel. So the pixel point is expanded so that each
                #location in the chessboard has at least one pixel in the final image. The results look okay, but the logic here is troubling me a bit.
                boolX = np.logical_and(p[:,:,0] >= x-.5, p[:,:,0]< x+1.5)
                boolY = np.logical_and(p[:,:,1] >= y-.5, p[:,:,1]< y+1.5)
                
                imVals = np.logical_and(boolX,boolY)
                #Projected image pixels that fall inside this pixel
                pxs = p[imVals,:] 
                #If there aren't any pixels set it to bg value
                if np.sum(imVals) == 0:
                    simImg[y,x] = bgVal
                elif pxs.shape[0] == 1:
                    simImg[y,x] = pxs[0,2]
                #Or if the points in this pixel all have the same value set it to that value
                elif np.mean(pxs[:,2]) == 0 or np.mean(pxs[:,2]) == 255:
                    simImg[y,x] = pxs[0,2]
                #Otherwise we're on an edge so. . .
                #Calculate weighted sum by distance to find the final color
                else:
                    #print("squirrly edge")
                    dist = np.sqrt((pxs[:,0]-x)**2 + (pxs[:,1]-y)**2) #Distance of each image pixel from the center of this pixel
                    invDist = 1/dist 
                    #Calculate weighted sum of each pixel
                    wtbydist = invDist/np.sum(invDist) * pxs[:,2]
                    #Weighted average to use
                    simImg[y,x] = np.sum(wtbydist)
                    #simImg[y,x] = np.mean(pxs[:,2])
    #Make the image RGB for ease of use
    #simImgRGB = np.dstack((simImg,simImg,simImg))
    return simImg
    
    
if __name__ == '__main__':
    
    
    squareSize = 6.7 #mm
    pxPermm = 3.8 #px/mm
    imSzPx = [2048,2048] #pixels
    stillImgDir = 'D:\\Work\\RoblyerLab\\trackDOSI\\plots\\simProbe'
    #imFOV = [imSzPx[0]/pxPermm, imSzPx[1]/pxPermm] #mm
    # stillImgDir  = 'D:\\Work\\RoblyerLab\\trackDOSI\\data\\trial1_left\\stills'
    # with open(os.path.join(stillImgDir,'selectedExtrinsics.pkl'), 'rb') as f:  
    #     saveRot, saveTrans = pickle.load(f)
    
    c = createChessboard((7,6),squareSize)
    numTrials = 1000
    zRotRange = [0,2*np.pi]
    xRotRange = [-np.pi/4,np.pi/4]
    yRotRange = [-np.pi/4,np.pi/4]
    
    xTransRange=[-110.0,110.0]
    yTransRange=[-110.0,110.0]
    zTransRange =[400.0,700.0] 
    it = 0
    
    probePos = np.zeros((3,numTrials))
    probeAngle =  np.zeros((3,numTrials))
    for i in range(0,numTrials):
        print("working on sim image %d of %d" % ((it+1),numTrials))
        
        thisRotX = np.round(xRotRange[0] + (xRotRange[1]-xRotRange[0]) * random.random(),2)
        thisRotY = np.round(yRotRange[0] + (yRotRange[1]-yRotRange[0]) * random.random(),2)
        thisRotZ = np.round(zRotRange[0] + (zRotRange[1]-zRotRange[0]) * random.random(),2)
        thisTransX = np.round(xTransRange[0] + (xTransRange[1]-xTransRange[0]) * random.random())
        thisTransY = np.round(yTransRange[0] + (yTransRange[1]-yTransRange[0]) * random.random())
        thisTransZ = np.round(zTransRange[0] + (zTransRange[1]-zTransRange[0]) * random.random())
        
        probePos[:,i] = np.array([thisTransX,thisTransY,thisTransZ])
        probeAngle[:,i] = np.array([thisRotX, thisRotY,thisRotZ])
        
      
        transPx = np.array([thisTransX,thisTransY,thisTransZ])
        # print(transPx)
        # print(probeAngle[:,i])
        #r = transformPts(c,[thisRotX,thisRotY,thisRotZ],transPx)
        #r = transformPts(c, [np.pi/4,0,0.0],[0,-100,1000.0])
        # transPx = np.array([0,0,thisTransZ])*pxPermm
        # r = transformPts(c,[np.pi/4,0,0],transPx)
        
        p = projectPts(c,np.array([thisRotX,thisRotY,thisRotZ]),transPx)
        #p = projectPts(c, np.array([0,0,0.0]),np.array([0,100,700.0]))
        
        # plt.pcolor(c[:,:,0],c[:,:,1],c[:,:,3])
        # plt.xlim((0,24))
        # plt.ylim((0,24))
        # plt.axis('equal')
        # plt.show()
        
        # plt.pcolor(r[:,:,0],r[:,:,1],r[:,:,3])
        # plt.axis('equal')
        # plt.show()
        
        plt.pcolor(p[:,:,0],p[:,:,1],p[:,:,2])
        plt.axis('equal')
        plt.xlim([0,2048])
        plt.ylim([0,2048])
        # plt.xlim([-imFOV[0]/2,imFOV[0]/2])
        # plt.ylim([-imFOV[1]/2,imFOV[1]/2])
        plt.show()
        
        simImg = getSimulatedImage(imSzPx,p)
        
        plt.pcolor(simImg)
        plt.axis('equal')
        plt.show()
        
        fname = 'simTranslation_%04d.tif' % it
        
        im = Image.new('L',(simImg.shape[1],simImg.shape[0]))
        im.putdata(simImg.ravel())
        im.save(os.path.join(stillImgDir,fname))
      
        it = it+1
        #plt.imshow(simImg,origin='lower')
 
    with open('probePosition.csv','w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['Center.X','Center.Y', 'Center.Z', 'Rot.X', 'Rot.Y', 'Rot.Z'])
        for row in range(numTrials):
            writer.writerow(np.array([probePos[:,row], probeAngle[:,row]]).flatten())
        
    
