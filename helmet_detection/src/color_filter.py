#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 29 11:19:43 2018

@author: kota421
"""

### USE ONLY redColorFilter_simple ##
###
# Functions to apply a color filter to static images

import numpy as np
import cv2
from matplotlib import pyplot as plt

def redColorFilter_simple(image):
    """
    Function to apply a color filter to an image.

    ASSUMES:
        -image: numpy array, cv2 image in BGR
    RETURNS:
        -comb_mask: numpy array of 0 and 1, 1 means the corresponding pixel in
            image is within the range (i.e. is red)
        -res: numpy array, resulting image
    """
    # Rotating image to change this from BGR to RGB
    rgb = image[...,::-1]
    # Change read image from RGB to HSV
    hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    # Apply threshold with red
    
    # define range of given color in HSV
    lower_color1 = np.array([0,100,100])
    upper_color1= np.array([10,255,255])
    lower_color2 = np.array([160,100,100])
    upper_color2 = np.array([180,255,255])


    # Color filter with each range
    mask1 = cv2.inRange(hsv, lower_color1, upper_color1)
    #print(mask1.dtype)
    mask2 = cv2.inRange(hsv,lower_color2,upper_color2)
    # combination of two masks
    comb_mask = np.logical_or(mask1,mask2)
    comb_mask = comb_mask.astype(np.uint8)

    res = cv2.bitwise_and(image,image,mask=comb_mask)
    return comb_mask, res

### Test with an image 
# =============================================================================
# #cv2.imshow("red part",red_part)
# cv2.namedWindow("mini red part", cv2.WINDOW_NORMAL) # Create window with freedom of dimensions                        # Read image
# imS = cv2.resize(res, (960, 540))                   # Resize image
# #cv2.imshow("mini red part", imS)
# cv2.namedWindow
# #cv2.imshow("res",res)                            # Show image
# cv2.waitKey(0)  
# =============================================================================






####~~~~~~~~~~~~~~~~~~ UNDER CONSTRUCTION ~~~~~~~~~~~~~~~~~~~~~~~~###

def redColorFilter_adaptive(IMG_NAME):
    # image is rgb, but cv2 understands it as bgr 
    img = cv2.imread(IMG_NAME,1)
    # so rotate the arraty to make it rgb, this allows matplotlib to graph normally
    rgb = img[...,::-1]
    gray = cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY) 
    mask = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                                 cv2.THRESH_BINARY,15,5)
    mask2 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)
    maskS = cv2.resize(mask,(960,540))
    cv2.imshow("adaptive",maskS)
    
    res = cv2.bitwise_and(img,img,mask)                    
    imS = cv2.resize(res, (960,540))                    # Resize image
    cv2.imshow("output", imS)                            # Show image
    cv2.waitKey(0)                                      
    
# =============================================================================
# t=redColorFilter_adaptive
# t('sumo.JPG')
# =============================================================================
    
def simpleThreshold(IMG_NAME):
    """
    Apply a simple thresholding to an image.
    Returns the applied image
    """
    # Read an image as a gray image
    img = cv2.imread(IMG_NAME,0)
    # apply simple threshoding
    ret, thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY) 
    res = cv2.bitwise_and(img,img,mask=thresh)
    cv2.imshow('threshold',res)
    cv2.waitKey(0)
    #return thresh
# =============================================================================
# IMG = 'redcurtain.jpg'
# cv2.imshow('img',simpleThreshold(IMG))
# img = cv2.imread('redcurtain.jpg',1)
# res = cv2.bitwise_and(img,img,mask=simpleThreshold(IMG))
# cv2.imshow('res',res)
# cv2.waitKey(0)
# =============================================================================



def adaptiveThreshold(IMG_NAME):
    """
    Apply an adaptive thresholding to the image
    """
    img = cv2.imread(IMG_NAME,0)
    img = cv2.resize(img,(300,400))
    img = cv2.medianBlur(img,5)
    
    ret,th1 = cv2.threshold(img,120,255,cv2.THRESH_BINARY)
    th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                cv2.THRESH_BINARY,11,2)
    th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,11,2)
    
    titles = ['Original Image', 'Global Thresholding (v = 127)',
                'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
    images = [img, th1, th2, th3]
    
    for i in range(4):
        plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])
    plt.show()
    
    for i in range(4):
        cv2.imshow(titles[i],images[i])
        cv2.waitKey(0)
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    