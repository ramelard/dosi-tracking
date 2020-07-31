# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 15:42:25 2020

@author: Matthew
"""

import csv
import numpy as np
import cv2
import matplotlib.pyplot as plt
#import time

def getSpreadImage(sigma):
    spreadImWidth = 10*sigma[1]
    spreadImHeight = 10*sigma[0]
    sX,sY = np.meshgrid(range(spreadImWidth), range(spreadImHeight))
    
    distFromCtr = np.sqrt((sX - np.round(spreadImHeight/2))**2 + (sY-np.round(spreadImWidth/2))**2)
    spreadIm = np.zeros(sX.shape)
    spreadIm[int(np.round(spreadImWidth/2)),int(np.round(spreadImHeight/2))] = 1
    
    spreadIm = cv2.GaussianBlur(spreadIm,(spreadImWidth-1,spreadImHeight-1),sigma[0],cv2.BORDER_DEFAULT)
    spreadIm[distFromCtr > (4 * sigma[1])] = 0;
    spreadIm = spreadIm/np.max(spreadIm);
    #plt.plot(spreadIm[:,int(spreadImWidth/2)])
    #plt.show()
    return spreadIm

def loadData(dataFile):
    xPx = []
    yPx = []
    mua = []
    mus = []
    with open(dataFile,'r') as f:
        rdr=csv.reader(f)
        next(rdr)
        for row in rdr:
            if not np.isnan(float(row[4])):
                xPx.append(int(np.round(float(row[4]))))
                yPx.append(int(np.round(float(row[5]))))
                mua.append(float(row[6]))
                mus.append(float(row[7]))
    
    return xPx, yPx, mua, mus

def getMap(xPx, yPx, dat2map, spreadIm, totalSize):
    singleMuaIm = np.zeros((totalSize[0],totalSize[1]),'float32')
    sumWtIm = np.zeros((totalSize[0],totalSize[1]),'float32')
    sumWtMuaIm = np.zeros((totalSize[0],totalSize[1]),'float32')
    countIm = np.zeros((totalSize[0],totalSize[1]),'float32')
    
    spreadImWidth = spreadIm.shape[1]
    spreadImHeight = spreadIm.shape[0]
    zeroSpread = np.zeros(spreadIm.shape)
    oneSpread = np.ones(spreadIm.shape)
    oneSpread[spreadIm <= 0] = 0
    #st = time.time()
    for i in range(len(dat2map)):
        if not np.isnan(xPx[i]):
            xi = xPx[i];
            yi = yPx[i];
            thisDat = dat2map[i]
            
            # cropLeftFlag = 0
            # cropRightFlag = 0
            # cropTopFlag = 0
            # cropBottomFlag = 0
            leftEdge = int(xi-spreadImWidth/2)
            
            # if leftEdge < 0:
            #     leftEdge = 0
            #     cropLeftFlag =1
            
            rightEdge = int(leftEdge+spreadImWidth)
            # if rightEdge >= imWidth:
            #     rightEdge = imWidth-1
            #     cropRightFlag =1 
            
            topEdge = int(yi-spreadImHeight/2)
            # if topEdge < 0:
            #     topEdge = 1
            #     cropTopFlag = 1
            
            bottomEdge = int(topEdge + spreadImHeight)
            # if bottomEdge >= imHeight:
            #     bottomEdge = imHeight-1;
            #     cropBottomFlag = 1;
            
            singleMuaIm[topEdge:bottomEdge,leftEdge:rightEdge] = spreadIm
            countIm[topEdge:bottomEdge,leftEdge:rightEdge] += oneSpread
            sumWtIm += singleMuaIm
            singleMuaIm *= thisDat
            sumWtMuaIm += singleMuaIm
            singleMuaIm[topEdge:bottomEdge,leftEdge:rightEdge] = zeroSpread
            
    with np.errstate(invalid='ignore'):       
        dat_im = sumWtMuaIm/sumWtIm
        
    dat_im[np.isnan(dat_im)] = 0  
    countIm[countIm > 5] = 5
    return dat_im, countIm

if __name__ == '__main__':
    dataFile = '../data/inclusion2xa_OPs.txt'
    bgImFile = '../data/inclusion2xa_img.tif'
    
   
    bgIm = cv2.imread(bgImFile)
    imHeight= bgIm.shape[0]
    imWidth = bgIm.shape[1]
    
    xPx,yPx,mua,mus = loadData(dataFile)
    
    sigma = (7,7)

    spreadIm = getSpreadImage(sigma)
        
    #plt.imshow(mua_im*255)
    mua_im, ctIm = getMap(xPx,yPx,mua,spreadIm,(imHeight,imWidth))
    nzpix = mua_im > 0;
    muaDisp = cv2.convertScaleAbs(255*(mua_im-mua_im[nzpix].min())/(mua_im.max()-mua_im[nzpix].min()))
    muaColor = cv2.applyColorMap(muaDisp, cv2.COLORMAP_JET);
    alphaChan = np.zeros(muaColor.shape)
    
    alphaChan[nzpix,:] = 0.3
    #wtIm[nzpix] += 2
    #scaleAlpha = (ctIm-ctIm.min())/(ctIm.max()-ctIm.min()) * (.7-.2) + .2
    #scaleAlpha[np.invert(nzpix)] = 0
    #scaleWtIm = np.tile(scaleAlpha[:,:,np.newaxis],(1,1,3))
    
    composite = bgIm * (1-alphaChan) + muaColor * (alphaChan)

    plt.imshow(ctIm)
    
    cv2.imshow('mua',composite.astype('uint8'))
    cv2.waitKey(0)
    cv2.destroyAllWindows()