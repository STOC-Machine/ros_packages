#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 12:53:35 2019

@author: kota421
"""
import cv2
def noise_cancel(color_filtered):
    """
    Function to apply noise canceling techniques to 
    to a given image using erosion.
    
    Assumes: 
        color_filtered: cv2.img, BGR
    Returns:
        noise_canceled: cv2.img
    """
    gray = cv2.cvtColor(color_filtered,cv2.COLOR_BGR2GRAY)
    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    erosion = cv2.erode(gray,kernel,iterations = 1)
    dilation = cv2.dilate(erosion,kernel,iterations = 1)
    noise_canceled = dilation
    return noise_canceled

