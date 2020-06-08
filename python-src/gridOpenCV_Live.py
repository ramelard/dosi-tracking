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

#initCam
#
#PURPOSE: Save a dictionary of camera info
#
#INPUTS:
#   camFile:       -NPZ file with the camera intrinsics
#   origin2sensor: -Distance from the chessboard origin to the measurement location
#   squareSize:    -Size of a chessboard square in mm
#   frameSize:     -Image size in pixels
#   boardShape:    -Shape of chessboard
#
#OUTPUTS:
#   camDict: -Dictionary with lots of handy camera parameters
#
def initCam(camFile, origin2sensor, squareSize, frameSize, boardShape):
    camDict = dict()
    #Read intrinsics from numpy npz file
    intrinsics = np.load(camFile)
    ##Some constants taken from the Matlab file
    camDict["cameraMatrix"] = intrinsics['cameraMatrix']
    camDict["distCoefs"] = intrinsics['distortionCoefs'][0] #Distortion coefficients
    camDict["origin2sensor"] = origin2sensor  # (mm) X-Distance from the top left grid point to the detector
    camDict["squareSize"] = squareSize
    ##############################################################
    #Other constants for convenience
    camDict["frameSize"] = frameSize
    camDict["cornerFactor"] = 0.5
    camDict["displayFactor"] = 0.5
    camDict["boardShape"] = boardShape
    camDict["criteria"] = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Criteria for subpixel localization of intersections
    #Location of the detector in "Relative" world coordinates
    
    #Get all of the grid points in "Relative" world coordinates
    camDict["boardPts"]=generateGridPoints(boardShape,squareSize)

    #Calculate distortion maps. The raw function undistort is pretty slow
    #Followed this guy's instructions: http://blog.nishihara.me/opencv/2015/09/03/how-to-improve-opencv-performance-on-lens-undistortion-from-a-video-feed/
    #Speeds it up a lot
    camDict["map1"],camDict["map2"]=cv2.initUndistortRectifyMap(camDict["cameraMatrix"],camDict["distCoefs"],np.eye(3),camDict["cameraMatrix"],(frameSize[0],frameSize[1]),5)
    
    return camDict
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
def getProbeLocation(cam,flatFrame):
  
    #Shrink the frame for finding corners
    flatFrameSmall = cv2.resize(flatFrame,(int(cam["frameSize"][0]*cam["cornerFactor"]),int(cam["frameSize"][1]*cam["cornerFactor"])))
    #Find corners on smaller frame
    patFound,corners=cv2.findChessboardCorners(flatFrameSmall,cam["boardShape"])
        
    #If a checkerboard is recognized
    if patFound:
        #Do subpixel localization
        #Corner locations are scaled up to correspond to full resolution frame
        corners2 = cv2.cornerSubPix(flatFrame,corners/cam["cornerFactor"],(11,11),(-1,-1),cam["criteria"])
            
        #Find the pose of the camera based on the corner locations
        #flags = 0 (or cv2.SOLVEPNP_ITERATIVE) is Iterative Levenberg-Marquardt
        #flags = 6 (or cv2.SOLVEPNP_IPPE) is a noniterative solution that depends on all the points being co-planar
        #doc link: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
        #Paper ref: T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation" 
        retVal, rvec,tvec=cv2.solvePnP(cam["boardPts"],corners2,cam["cameraMatrix"],0,flags=cv2.SOLVEPNP_IPPE)
                            
        #Calculate the rotation matrix for this image
        rotMat = cv2.Rodrigues(rvec)[0]
        #Location in world coordinates
        P_A = np.vstack((rotMat.dot(np.array(cam["origin2sensor"]).reshape(3,1)) + tvec.reshape(3,1),1))
        #Location in world coordinates relative to origin of first image
        #rotMat0 = cv2.Rodrigues(rvec)[0] #Turn rotation vector into matrix
        #     #Make matrix 4x4
        #rotMat4x4 = np.vstack((np.hstack((rotMat0,np.array([0,0,0]).reshape(3,1))),np.array([0,0,0,1]).reshape(1,4)))
        #tvec0 = tvec #Save translation vector
        #Make translation matrix 4x4
        #tMat4x4 = np.vstack((np.hstack((np.zeros((3,3)),tvec.reshape(3,1))),np.array([0,0,0,0]).reshape(1,4)))
        #Combine both matricies
        #M0 is the transformation from World space to Camera Space
        #M0 = rotMat4x4 + tMat4x4
        #     #Invert M0 which is now the transformation from Camera Space to World Space
        #     M0_inv = np.linalg.pinv(M0)
        #rotMat4x4 = np.vstack((nphstack((rotmat,np.array([0,0,0]).reshape(3,1)))))
        #M0 = 
       # P_B = M0_inv.dot(P_A)
        #P_B = M0_inv.dot(P_A)
        return P_A,rvec,tvec
    else:
        return -1,-1,-1
     
        

#Entry point when you run the file Currently saves an mp4 video
#of the first 120 frames
#Also stores the trajectory of the probe
#Will probably roll this into a function in the near future
if __name__ == '__main__':
    cam=initCam('cameraParams.npz',(0,0,0),10,(1280,720),(4,3))    
    p1 = np.array([0, 0, 0],dtype=float)
    p2 = np.array([40, 0, 0],dtype=float)
    p3 = np.array([0, 40, 0],dtype=float)
    p4 = np.array([0, 0, -40],dtype=float)
    frameCol = np.zeros((int(cam["frameSize"][1]*cam["displayFactor"]),int(cam["frameSize"][0]*cam["displayFactor"]) ,3),dtype='uint8')
    
    #Initialize webcam and set frame size   
    cap = cv2.VideoCapture(0)
    cap.set(3, cam["frameSize"][0])
    cap.set(4, cam["frameSize"][1])
    #Set the number of frames to save
    saveFrames = 120
    #allocate memory for saving video
    saveVid = np.zeros((int(cam["frameSize"][1]*cam["displayFactor"]),int(cam["frameSize"][0]*cam["displayFactor"]),3,saveFrames),dtype='uint8')
    #Good frame number
    goodFrames = 0
    frameNum = -1
    firstTime = True
    sensorPathWorld = []
    sensorPathImage = []
    sensorMask = []
    #M0_inv = np.eye(4)
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
        flatFrame=cv2.remap(gray,cam["map1"],cam["map2"], interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        P_B,rvec,tvec = getProbeLocation(cam,flatFrame)
        if not isinstance(P_B,int): #If this is the first frame do some calculations
            #     firstTime = False #make sure this only runs once
            #     firstIm = flatFrame #Save the first image because why not
            #     rvec0 = rvec #Save the rotation vector
          
        
            #Save the sensor path in world coordinates
            sensorPathWorld.append(P_B[0:3].reshape(3))
            #Project the overlay points in camera space
            imgpts, _ = cv2.projectPoints(cam["origin2sensor"], rvec, tvec, cam["cameraMatrix"], 0)
            imgp1, _ = cv2.projectPoints(p1, rvec, tvec, cam["cameraMatrix"], 0)
            imgp2, _ = cv2.projectPoints(p2, rvec, tvec, cam["cameraMatrix"], 0)
            imgp3, _ = cv2.projectPoints(p3, rvec, tvec, cam["cameraMatrix"], 0)
            imgp4, _ = cv2.projectPoints(p4, rvec, tvec, cam["cameraMatrix"], 0)
            ptXInt = int(np.floor(imgpts[0,0,0]*cam["displayFactor"])) #Cast to integer so it can be drawn
            ptYInt = int(np.floor(imgpts[0,0,1]*cam["displayFactor"]))
            #Save sensor path in the image for visuilization
            sensorPathImage.append([ptXInt,ptYInt])
            sensorMask.append(True)
            #Rescale image and convert to color for display
            frameCol = cv2.cvtColor(cv2.resize(flatFrame,(int(cam["frameSize"][0]*cam["displayFactor"]),int(cam["frameSize"][1]*cam["displayFactor"]))),cv2.COLOR_GRAY2BGR)
            #Display axes on the screen
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*cam["displayFactor"])),int(np.floor(imgp1[0,0,1]*cam["displayFactor"]))),
                     (int(np.floor(imgp2[0,0,0]*cam["displayFactor"])),int(np.floor(imgp2[0,0,1]*cam["displayFactor"]))),(0,0,255),3)
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*cam["displayFactor"])),int(np.floor(imgp1[0,0,1]*cam["displayFactor"]))),
                     (int(np.floor(imgp3[0,0,0]*cam["displayFactor"])),int(np.floor(imgp3[0,0,1]*cam["displayFactor"]))),(0,255,0),3)
            cv2.line(frameCol,(int(np.floor(imgp1[0,0,0]*cam["displayFactor"])),int(np.floor(imgp1[0,0,1]*cam["displayFactor"]))),
                     (int(np.floor(imgp4[0,0,0]*cam["displayFactor"])),int(np.floor(imgp4[0,0,1]*cam["displayFactor"]))),(255,0,0),3)
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
    
    pathXImage = [x[0]/cam["displayFactor"] for x in sensorPathImage]
    pathYImage = [x[1]/cam["displayFactor"] for x in sensorPathImage]
    
    #Rudimentary plots
   
    
    plt.figure()
    plt.imshow(frameCol)
    
    plt.figure()
    plt.plot(pathXImage, pathYImage,'o')
    plt.plot(pathXImage[0],pathYImage[0],'o')
    plt.title("Path in Image space")
    
    plt.figure()
    plt.plot(pathXWorld, pathYWorld,'o')
    plt.plot(pathXWorld[0],pathYWorld[0],'o')
    plt.title("Path in world space")
    plt.axis('equal')
    
    height,width,depth, frames = saveVid.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    v = cv2.VideoWriter('test.mp4',fourcc,20,(width, height))
    for i in range(frames):
        v.write(saveVid[:,:,:,i])
    
    v.release()
    
  

