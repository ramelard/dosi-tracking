# -*- coding: utf-8 -*-
"""
Tracks a 2D chessboard pattern in 3D space
Displays results live using an attached video device (ie webcam)

Runs at 10-20 fps on my laptop

Created on Mon May 18 2020

@author: Matthew
"""
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

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

#Entry point when you run the file Currently saves an mp4 video
#of the first 120 frames
#Also stores the trajectory of the probe
#Will probably roll this into a function in the near future
if __name__ == '__main__':
    #Read intrinsics from numpy npz file
    intrinsics = np.load('cameraParams.npz')
    ##Some constants taken from the Matlab file
    cameraMatrix = intrinsics['cameraMatrix']
    distCoefs = intrinsics['distortionCoefs'][0] #Distortion coefficients
    origin_to_sensorX = -34  # (mm) X-Distance from the top left grid point to the detector
    origin_to_sensorY = 28  # (mm) Y-Distance from top left grid point to detector
    probe_depth = 67.3  # (mm) Z-Distance from top left grid point to detector
    squareSize = 10 # (mm) Size of each square in the checkerboard
    #fps = 30 # frames per second of video
    ##############################################################
    #Other constants for convenience
    frameWidth = 1280 #(px) Width of video frame
    frameHeight = 720 #(px) Height of video frame
    findCornersFactor = .5 #Finding checkerboard corners is slow, so scale down the image by this factor prior to looking for them
    displayFactor = .5 #The images are too big to fit on my screen, so scale them down by this much before showing
    boardShape = (4,3) #Number of intersections on the checkerboard along X and Y
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Criteria for subpixel localization of intersections
    #Location of the detector in "Relative" world coordinates
    sensorLoc = np.array([origin_to_sensorX, origin_to_sensorY, probe_depth])
    #Locations outlining probe
    p1 = np.array([0, 0, 0],dtype=float)
    p2 = np.array([40, 0, 0],dtype=float)
    p3 = np.array([0, 40, 0],dtype=float)
    p4 = np.array([0, 0, -40],dtype=float)
    #p2 = np.array([-38, 0, 0],dtype=float)
    #p3 = np.array([-38, 45, 0],dtype=float)
    #p4 = np.array([-38, 45, probe_depth],dtype=float)
#  plot([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)], '.-g', 'linewidth', 2, 'markersize', 20)
    
    #Get all of the grid points in "Relative" world coordinates
    g=generateGridPoints(boardShape,squareSize)

   #Preallocate memory for some variables (this will fail if the video is longer than 2 minutes)
    sensorPathWorld = []#np.zeros((len(f),3)) #Path of sensor in World coordinates relative to upper left intersection on frame 1
    sensorPathImage = []#np.zeros((len(f),2)) #Path of detector in Camera coordinates
    sensorMask = []#np.ones(len(f),dtype=bool) #Logical vector saying whether or not probe was detected on that frame
    vidIdx = 0 #Current video index. Videos were split into multiple files
    frameNum = -1 #This variable will store the current frame number (it gets incremented before being used which is why it's not 0)
    
    #start = time.time() #Start time
    firstTime = True #The first frame has some additional processing associated with it

    #Calculate distortion maps. The raw function undistort is pretty slow
    #Followed this guy's instructions: http://blog.nishihara.me/opencv/2015/09/03/how-to-improve-opencv-performance-on-lens-undistortion-from-a-video-feed/
    #Speeds it up a lot
    map1,map2=cv2.initUndistortRectifyMap(cameraMatrix,distCoefs,np.eye(3),cameraMatrix,(frameWidth,frameHeight),5)
    frameCol = np.zeros((int(frameHeight*displayFactor),int(frameWidth*displayFactor) ,3),dtype='uint8')
    
    #Initialize webcam and set frame size   
    cap = cv2.VideoCapture(vidIdx)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    #Set the number of frames to save
    saveFrames = 120
    #allocate memory for saving video
    saveVid = np.zeros((int(frameHeight*displayFactor),int(frameWidth*displayFactor),3,saveFrames),dtype='uint8')
    #Good frame number
    goodFrames = 0
    #Elapsed time for a frame
    fel = 0.0
    while(1):
        fstart = time.time()
        ret,frame = cap.read() #Grab a frame
        # Check success
        if not cap.isOpened():
            raise Exception("Could not open video device")
        #Increment frame counter
        frameNum = frameNum + 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert to grayscale
        #This is the second half of the undistortion function that is pretty fast
        flatFrame=cv2.remap(gray,map1,map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        
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
            sensorPathWorld.append(P_B[0:3].reshape(3))
            #Project the overlay points in camera space
            imgpts, _ = cv2.projectPoints(P_B[0:3], rvec0, tvec0, cameraMatrix, 0)
            imgp1, _ = cv2.projectPoints(p1, rvec, tvec, cameraMatrix, 0)
            imgp2, _ = cv2.projectPoints(p2, rvec, tvec, cameraMatrix, 0)
            imgp3, _ = cv2.projectPoints(p3, rvec, tvec, cameraMatrix, 0)
            imgp4, _ = cv2.projectPoints(p4, rvec, tvec, cameraMatrix, 0)
            ptXInt = int(np.floor(imgpts[0,0,0]*displayFactor)) #Cast to integer so it can be drawn
            ptYInt = int(np.floor(imgpts[0,0,1]*displayFactor))
            #Save sensor path in the image for visuilization
            sensorPathImage.append([ptXInt,ptYInt])
            sensorMask.append(True)
            #Rescale image and convert to color for display
            frameCol = cv2.cvtColor(cv2.resize(flatFrame,(int(frameWidth*displayFactor),int(frameHeight*displayFactor))),cv2.COLOR_GRAY2BGR)
            #Display axes on the screen
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*displayFactor)),int(np.floor(imgp1[0,0,1]*displayFactor))),
                     (int(np.floor(imgp2[0,0,0]*displayFactor)),int(np.floor(imgp2[0,0,1]*displayFactor))),(0,0,255),3)
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*displayFactor)),int(np.floor(imgp1[0,0,1]*displayFactor))),
                     (int(np.floor(imgp3[0,0,0]*displayFactor)),int(np.floor(imgp3[0,0,1]*displayFactor))),(0,255,0),3)
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*displayFactor)),int(np.floor(imgp1[0,0,1]*displayFactor))),
                     (int(np.floor(imgp4[0,0,0]*displayFactor)),int(np.floor(imgp4[0,0,1]*displayFactor))),(255,0,0),3)
            #Save the first few frames
            if goodFrames < saveFrames:
                saveVid[:,:,:,goodFrames] = frameCol
                goodFrames = goodFrames + 1
            #Calculate the time spent processing this frame
            fel = time.time() - fstart
        #If the grid pattern isn't found, do nothing and mark the sensor mask
        else:
            sensorMask.append(False)
        
        #If the framerate is too high, sleep until it's time to display
        #This should probably be replaced by some double buffering scheme
        if fel < 0.05:
            time.sleep(0.05-fel)
            fel = 0.05
        #Write FPS on top of display    
        cv2.putText(frameCol,'FPS: %.3f' % (1/fel), 
            (10,20), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1,
            (0,0,255),
            1)
        #Show the frame
        cv2.imshow('frame',frameCol)
        #Press q to quit out of the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cap.release()
    cv2.destroyAllWindows()
    
    #Split up path variables
    pathXWorld = [x[0] for x in sensorPathWorld]
    pathYWorld = [x[1] for x in sensorPathWorld]
    pathZWorld = [x[2] for x in sensorPathWorld]
    
    pathXImage = [x[0]/displayFactor for x in sensorPathImage]
    pathYImage = [x[1]/displayFactor for x in sensorPathImage]
    
    #Rudimentary plots
   
    
    plt.figure()
    plt.imshow(frameCol)
    
    plt.figure()
    plt.plot(pathXImage, pathYImage)
    
    height,width,depth, frames = saveVid.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    v = cv2.VideoWriter('test.mp4',fourcc,20,(width, height))
    for i in range(frames):
        v.write(saveVid[:,:,:,i])
    
    v.release()
    
  

