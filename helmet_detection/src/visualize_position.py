#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 11:36:06 2019

@author: kota421
"""

import cv2
from helmet_detection import test
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

def visualize_position():
    X=[]
    Y=[]
    cap = cv2.VideoCapture(0)
    stop = None
    
    while not stop:
        # Read a frame
        _, frame = cap.read()
        # Add the center point of a red object if it exists
        img,x,y = test(frame)
        if x != None:
            X.append(x)
            Y.append(y)
        # Plot it
        plt.xlim(0,img.shape[2])
        plt.ylim(0,img.shape[1])
        plt.scatter(X,Y)
        plt.show()
        plt.clf()
        time.sleep(0.5)
        # Press any key to stop
        stop = input()

visualize_position()
