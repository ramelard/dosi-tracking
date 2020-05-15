# -*- coding: utf-8 -*-
"""
Code that will track position of a grid of points in 3d and use that to determine the physical location
of the dDOSI sensor

Created on Wed May 13 16:26:10 2020

@author: Matthew
"""
import numpy as np
import cv2
import os
import glob
import time
from matplotlib import pyplot as plt
from scipy.io import loadmat

#generateGridPoints
#
#PURPOSE: Generates evenly spaced points on a grid in WORLD COORDINATES with 0,0 set as the top left intersection
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

#Entry point when you run the file
if __name__ == '__main__':
    ##Some constants taken from the Matlab file
    cameraMatrix = np.matrix([[2858.2,0,1051.3],[0,2849.7,1064.8],[0,0,1]]) #Camera intrinsics
    distCoefs = np.array([-.2499,.2871,0,0]) #Distortion coefficients
    origin_to_sensorX = -34  # (mm) X-Distance from the top left grid point to the detector
    origin_to_sensorY = 28  # (mm) Y-Distance from top left grid point to detector
    probe_depth = 67.3  # (mm) Z-Distance from top left grid point to detector
    squareSize = 6.5 # (mm) Size of each square in the checkerboard
    fps = 30 # frames per second of video
    ##############################################################
    #Other constants for convenience
    frameWidth = 2048 #(px) Width of video frame
    frameHeight = 2048 #(px) Height of video frame
    findCornersFactor = .25 #Finding checkerboard corners is slow, so scale down the image by this factor prior to looking for them
    displayFactor = .5 #The images are too big to fit on my screen, so scale them down by this much before showing
    boardShape = (7,6) #Number of intersections on the checkerboard along X and Y
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Criteria for subpixel localization of intersections
    #Location of the detector in "Relative" world coordinates
    sensorLoc = np.array([origin_to_sensorX, origin_to_sensorY, probe_depth])
    #Locations outlining probe
    p1 = np.array([0, 0, 0],dtype=float)
    p2 = np.array([-38, 0, 0],dtype=float)
    p3 = np.array([-38, 45, 0],dtype=float)
    p4 = np.array([-38, 45, probe_depth],dtype=float)
#  plot([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)], '.-g', 'linewidth', 2, 'markersize', 20)
    
    #Directory with AVI files
    dataDir = 'D:\\Work\\RoblyerLab\\trackDOSI\\data\\hand1'
    f=glob.glob(os.path.join(dataDir,'fc2*.avi')) #list of avi files
    #Get all of the grid points in "Relative" world coordinates
    g=generateGridPoints(boardShape,squareSize)

   #Preallocate memory for some variables (this will fail if the video is longer than 2 minutes)
    sensorPathWorld = np.zeros((120*fps,3)) #Path of sensor in World coordinates relative to upper left intersection on frame 1
    sensorPathImage = np.zeros((120*fps,2)) #Path of detector in Camera coordinates
    sensorMask = np.ones(120*fps,dtype=bool) #Logical vector saying whether or not probe was detected on that frame
    vidIdx = 0 #Current video index. Videos were split into multiple files
    frameNum = -1 #This variable will store the current frame number (it gets incremented before being used which is why it's not 0)
    
    start = time.time() #Start time
    firstTime = True #The first frame has some additional processing associated with it
    flatFrame = np.zeros((frameWidth,frameHeight),dtype = 'uint8') #Allocate memory for the flattened frame
    #Calculate distortion maps. The raw function undistort is pretty slow
    #Followed this guy's instructions: http://blog.nishihara.me/opencv/2015/09/03/how-to-improve-opencv-performance-on-lens-undistortion-from-a-video-feed/
    #Speeds it up a lot
    map1,map2=cv2.initUndistortRectifyMap(cameraMatrix,distCoefs,np.eye(3),cameraMatrix,(frameWidth,frameHeight),cv2.CV_16SC2)
    
    #Loop through all the video files
    while vidIdx < len(f):
        #Open the video file
        cap = cv2.VideoCapture(f[vidIdx])
        #Loop through all the frames
        while(cap.isOpened()):
            ret, frame = cap.read() #Grab a frame
            #If there are no more frames left to grab then break out of this loop
            if not ret:
                break
            #Increment frame counter
            frameNum = frameNum + 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert to grayscale
            #This is the second half of the undistortion function that is pretty fast
            cv2.remap(gray,map1,map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,dst=flatFrame)
            
            #Shrink the frame for finding corners
            flatFrameSmoll = cv2.resize(flatFrame,(int(frameWidth*findCornersFactor),int(frameHeight*findCornersFactor)))
            #Find corners on smaller frame
            patFound,corners=cv2.findChessboardCorners(flatFrameSmoll,boardShape)
            
            #If a checkerboard is recognized
            if patFound:
                #Do subpixel localization
                #Corner locations are scaled up to correspond to full resolution frame
                corners2 = cv2.cornerSubPix(flatFrame,corners/findCornersFactor,(11,11),(-1,-1),criteria)
                
                #Find the pose of the camera based on the corner locations
                #flags = 0 (or cv2.SOLVEPNP_ITERATIVE) is Iterative Levenberg-Marquardt
                #flags = 6 (or cv2.SOLVEPNP_IPPE) is a noniterative solution that depends on all the points being co-planar
                #doc link: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
                #Paper ref: T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation" 
                retVal, rvec,tvec=cv2.solvePnP(g,corners2,cameraMatrix,0,flags=cv2.SOLVEPNP_IPPE)
                
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
                #Project the detector point in world coordinates into Camera Space for visualization
                imgpts, _ = cv2.projectPoints(P_B[0:3], rvec0, tvec0, cameraMatrix, 0)
                imgp1, _ = cv2.projectPoints(p1, rvec, tvec, cameraMatrix, 0)
                imgp2, _ = cv2.projectPoints(p2, rvec, tvec, cameraMatrix, 0)
                imgp3, _ = cv2.projectPoints(p3, rvec, tvec, cameraMatrix, 0)
                imgp4, _ = cv2.projectPoints(p4, rvec, tvec, cameraMatrix, 0)
                ptXInt = int(np.floor(imgpts[0,0,0]*displayFactor)) #Cast to integer so it can be drawn
                ptYInt = int(np.floor(imgpts[0,0,1]*displayFactor))
                #Save sensor path in the image for visuilization
                sensorPathImage[frameNum,:]=[ptXInt,ptYInt]
                #Rescale image and convert to color for display
                frameCol = cv2.cvtColor(cv2.resize(flatFrame,(int(frameWidth*displayFactor),int(frameHeight*displayFactor))),cv2.COLOR_GRAY2BGR)
                #Plot grid corners as circles
                #for i in range(len(corners2)):
                cv2.circle(frameCol,(int(np.floor(corners2[0,0,0]*displayFactor)),int(np.floor(corners2[0,0,1]*displayFactor))),10,(0,0,255),2)
                #cv2.drawChessboardCorners(frameCol,(7,6),corners2/2,patFound)
                cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*displayFactor)),int(np.floor(imgp1[0,0,1]*displayFactor))),
                         (int(np.floor(imgp2[0,0,0]*displayFactor)),int(np.floor(imgp2[0,0,1]*displayFactor))),(0,255,0),3)
                cv2.line(frameCol,(int(np.floor(imgp2[0,0,0]*displayFactor)),int(np.floor(imgp2[0,0,1]*displayFactor))),
                         (int(np.floor(imgp3[0,0,0]*displayFactor)),int(np.floor(imgp3[0,0,1]*displayFactor))),(0,255,0),3)
                cv2.line(frameCol,(int(np.floor(imgp3[0,0,0]*displayFactor)),int(np.floor(imgp3[0,0,1]*displayFactor))),
                         (int(np.floor(imgp4[0,0,0]*displayFactor)),int(np.floor(imgp4[0,0,1]*displayFactor))),(0,255,0),3)
  
                #Draw circle at detector location
                cv2.circle(frameCol,(ptXInt,ptYInt),10,(255,0,0),-1)
            #If the grid pattern isn't found, do nothing and mark the sensor mask
            else:
                sensorMask[frameNum] = False
            
            #Show the frame
            cv2.imshow('frame',frameCol)
            #Press q to quit out of the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        vidIdx = vidIdx + 1
        cap.release()
    cv2.destroyAllWindows()
    #Get total time and calculate FPS
    el = time.time() - start
    print("Total time: %.3f" % el)
    print("FPS: %.3f" % ((frameNum+1)/el))
    
    #Crop sensor paths to actual number of frames
    sensorPathWorld = sensorPathWorld[0:frameNum+1,:]
    sensorPathImage = sensorPathImage[0:frameNum+1,:]
    sensorMask = sensorMask[0:frameNum+1]
    
    #Split up path variables
    pathXWorld = [x[0] for x in sensorPathWorld]
    pathYWorld = [x[1] for x in sensorPathWorld]
    pathZWorld = [x[2] for x in sensorPathWorld]
    
    pathXImage = [x[0]/displayFactor for x in sensorPathImage]
    pathYImage = [x[1]/displayFactor for x in sensorPathImage]
    
    #Rudimentary plots
   
    
    plt.figure()
    plt.imshow(frameCol)
    plt.plot(pathXImage, pathYImage)
    
    dat = loadmat(os.path.join(dataDir,'track_probe_variables-0515_1554.mat'))
    matlab3d=dat['sensor3D']
    matlab2d = dat['sensor_route']
    
    plt.figure()   
    plt.plot(pathXWorld,pathYWorld,'o')
    plt.plot(matlab3d[0,:],matlab3d[1,:],'o')
    plt.legend(('Python','Matlab'))
    plt.xlabel('X-Position (mm)')
    plt.ylabel('Y-Position (mm)')
    plt.title('World coordinates of sensor')
    plt.savefig('PositionOverlay.png')
    
    plt.figure()
    plt.plot(pathXImage,pathYImage,'o')
    plt.plot(matlab2d[0,:],matlab2d[1,:])
    
    diffX = pathXWorld - matlab3d[0,:]
    diffY = pathYWorld - matlab3d[1,:]
    diffZ = pathZWorld - matlab3d[2,:]
    
    totalDiff = np.sqrt(diffX**2 + diffY**2 + diffZ**2)
    
    t = np.arange(len(diffX))/fps
    

    fig,ax=plt.subplots(3,1,sharex=True,figsize = (7,10))
    ax[0].plot(t,diffX)
    ax[0].set(ylabel='X-Difference (mm)',title='Matlab vs. Python')
    ax[1].plot(t,diffY)
    ax[1].set(ylabel='Y-Difference (mm)')
    ax[2].plot(t,diffZ)
    ax[2].set(xlabel='Time (s)',ylabel='Z-Difference (mm)')
    plt.savefig('PositionDifference.png')

  

