# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 14:55:52 2020

@author: Matthew
"""
import csv
import glob
import numpy as np
import os
from PIL import Image
import random
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import Sequential
from tensorflow.keras.layers import Conv2D
from tensorflow.keras.layers import MaxPooling2D
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import Flatten
from matplotlib import pyplot as plt
#Want to try to estimate position and angular rotation of images using a neural network

#Directory with simulated images
imDir = 'D:\\Work\\RoblyerLab\\trackDOSI\\plots\\simProbe'
#File with labels for images
dataFile = 'D:\\googleDrive\\dDOSI\\trackDOSI\\code\\trackGrid\\code\\probePosition.csv'

#Set seed for reproducibility
tf.random.set_seed(14850)
random.seed(14850)

#Calculate number of images
fList = glob.glob(os.path.join(imDir,'*.tif'))
numIms = len(fList)
#Preallocate for labels
data = np.zeros((numIms,6))
#Load CSV into matrix
with open(dataFile,newline='') as f:
    reader = csv.reader(f)
    line_ct = 0
    for row in reader:
        if line_ct != 0:
            data[line_ct-1,:] = row
        line_ct = line_ct+1
        
#Gather all images into a big 'ole matrix (numIms, rows, columns, channels)
imRes = np.array([640,360])
imMatrix = np.zeros((numIms,imRes[1],imRes[0],1),dtype='uint8')
imNum = 0
for fname in fList:
    imMatrix[imNum,:,:,0] = np.array(Image.open(fname))
    imNum = imNum+1
     
# Split the data into training and testing sets
testSplit = 0.1
numTest = int(np.round(testSplit*numIms))
numTrain = numIms-numTest
idxs = range(numIms)
testIdxs = random.sample(idxs,numTest)
testBool = np.zeros(numIms,'bool')
testBool[testIdxs] = True
trainBool = np.logical_not(testBool)

testIms= np.zeros((numTest,imRes[1],imRes[0],1))
trainIms = np.zeros((numTrain,imRes[1],imRes[0],1))
testIms[:,:,:,0] = imMatrix[testBool,:,:,0]
trainIms[:,:,:,0] = imMatrix[trainBool,:,:,0]
testLabels = data[testBool,0:3]
trainLabels = data[trainBool,0:3]


#Convolutional neural network model
model = Sequential()
model.add(Conv2D(16, (8,8), activation='relu', input_shape=(imRes[1],imRes[0],1)))
model.add(MaxPooling2D((2,2)))
model.add(Conv2D(32,(4,4),activation='relu'))
model.add(MaxPooling2D((2,2)))
model.add(Conv2D(32,(4,4),activation='relu'))
model.add(MaxPooling2D((2,2)))
model.add(Conv2D(16,(8,8),activation='relu'))
model.add(MaxPooling2D((2,2)))
model.add(Flatten())
model.add(Dense(40,activation='relu'))
model.add(Dense(30,activation='relu'))
model.add(Dense(20,activation='relu'))
model.add(Dense(10,activation='relu'))
model.add(Dense(3, activation='linear'))
model.summary()

model.compile(loss='mse',
              optimizer='adam',
              metrics=['mse','mae'])

model.fit(trainIms, trainLabels, epochs=1000,verbose=1)

test_loss,test_mse,test_mae = model.evaluate(testIms,testLabels)
print('Test accuracy: ', test_mse)

predX = model.predict(testIms)
trainPred = model.predict(trainIms)

fig = plt.figure()
plt.plot(testLabels[:,0],predX[:,0],'o')
plt.plot(testLabels[:,1],predX[:,1],'o')
plt.plot(testLabels[:,2],predX[:,2],'o')
#plt.plot([0, imSize], [0,imSize],'k--')
plt.legend(["X","Y","Z"])
plt.xlabel('Known Location (mm)')
plt.ylabel('Predicted Location (mm)')
plt.title('Neural Network Test Accuracy')


fig = plt.figure()
plt.plot(trainLabels[:,0],trainPred[:,0],'o')
plt.plot(trainLabels[:,1],trainPred[:,1],'o')
plt.plot(testLabels[:,2],predX[:,2],'o')
#plt.plot([0, imSize], [0,imSize],'k--')
plt.legend(["X","Y","Z"])
plt.xlabel('Known Location (mm)')
plt.ylabel('Predicted Location (mm)')
plt.title('Neural Network Train Accuracy')

# diffX = dataTest["centerX"] - predX[:,0]
# diffY = dataTest["centerY"] - predX[:,1]

# fig = plt.figure()
# plt.plot(dataTest["thetaRot"],diffX,'o')
# plt.plot(dataTest["thetaRot"],diffY,'o')
# plt.plot([0,2*np.pi],[0,0],'k--')
# plt.xlabel(r'$/theta$ (rad)')
# plt.ylabel('DNN - actual (px)')
# plt.title('Accuracy with Theta')

# fig = plt.figure()

# plt.plot(dataTest["phiRot"],diffX,'o')
# plt.plot(dataTest["phiRot"],diffY,'o')
# plt.plot([0,np.pi/2],[0,0],'k--')
# plt.xlabel(r'$\phi$ (rad)')
# plt.ylabel('DNN - actual (px)')
# plt.title('Accuracy with Phi')

# fig = plt.figure()

# plt.plot(dataTest["probeRad"],diffX,'o')
# plt.plot(dataTest["probeRad"],diffY,'o')
# plt.plot([25,35],[0,0],'k--')
# plt.xlabel('probe radius (px)')
# plt.ylabel('DNN - actual (px)')
# plt.title('Accuracy with probeRadius')

