#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 11 20:58:34 2019

@author: kota421
"""

def move2object(IMG_SHAPE,x,y,h,w):
    """
    Function to tell the drone in which direction and with how much
    distance it should move in order to centerlize the object in its
    vision.
    
    <FRAME DIRECTIONS>
    2D directions in the drone's frame in terms of the XY directions 
    in the image of shape IMG_SHAPE:
        forward: -y
        back: +y 
        right: +x
        left: -x
    Plus, vertical directions are defined as follows:
        up: +z
        down: -z
    
    <ARGUMENTS>
    Assumes:
        -IMG_SHAPE: tuple of two ints (height,width)
        -x,y: int, coordinates of the center of the object
        -h,w: int, height and width of the object
    Returns:
        -(x-direction,delta_x*): x-direction is str, either 'right' or 'left' or '+' **
        -(y-direction,delta_y): y-direction is str, either 'forward' or 'back' or '+'
        -(z-direction,delta_z): z-direction is str, either 'up' or 'down' or '+'
            * delta_x is a float representing the ratio of the separation of the center
                of the image and the center of the object in x direction. 
                (-1.0 < delta_x < 1.0)
            ** '+' means that the drone should keep the current position in that axis
                delta_x is 0 in this case.
    """
    assert x>=0 and y>=0
    import numpy as np
    H,W = IMG_SHAPE[0], IMG_SHAPE[1]
    # Center of the image
    O = np.array([H/2, W/2])
    # Center of the object
    P = np.array([y,x])
    OP = P - O
    OP_y = list(OP)[0]
    OP_x = list(OP)[1]
    # Determines the possible range of the distance that the drone will keep with 
    #   respect to the object
    a,b = 0.2, 0.3
    c,d = a,b
    
    x_frac = OP_x/(W/2)
    y_frac = OP_y/(H/2)
    # X
    if x_frac > 0.05:
        (x_direction,delta_x) = 'right', x_frac 
    elif x_frac < -0.05:
        (x_direction,delta_x) = 'left', x_frac
    else:
        (x_direction,delta_x) = '+', 0
    # Y
    if y_frac > 0.05:
        (y_direction,delta_y) = 'back', y_frac
    elif y_frac < -0.05:
        (y_direction,delta_y) = 'forward', y_frac
    else:
        (y_direction,delta_y) = '+', 0
    # Z
    if h/H < a or w/W < c:
        (z_direction,delta_z) = 'down', h/H - a
    elif h/H > b or w/W > d:
        (z_direction,delta_z) = 'up', h/H - b
    else:
        (z_direction,delta_z) = '+', 0
 
    return (x_direction,delta_x),(y_direction,delta_y),(z_direction,delta_z)

### TEST

## Variables
H = 2.0
W = 2.0
IMG_SHAPE = (H,W)
x = 1.15
y = 0.9
z = 7.00
h = 3/z
w = 2/z

    
## Visualize
def showMeHelmet(IMG_SHAPE,x,y,h,w):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    H, W = IMG_SHAPE[0], IMG_SHAPE[1]
    if h*w > 0:
        delta_x = x - (W/2)
        delta_y = y - (H/2)
        plt.figure()
        fig = plt.gca()
        plt.scatter([delta_x],[delta_y],color='black')
        obj = Rectangle((delta_x-(w/2),delta_y-(h/2)), w,h,alpha=0.5,facecolor='red')
        fig.add_patch(obj)
    plt.xlim(-W/2,W/2)
    plt.ylim(H/2,-H/2)
    plt.scatter([0],[0],marker='+')
    plt.show()
    
#showMeHelmet(IMG_SHAPE,x,y,h,w)


## Info for Humans
def moveDrone(IMG_SHAPE,x,y,h,w):
    H, W = IMG_SHAPE[0], IMG_SHAPE[1]
    (x_direction,delta_x),(y_direction,delta_y),(z_direction,delta_z) = move2object(IMG_SHAPE,x,y,h,w)
    print("Image Width and Height:",W,H)
    print("Object Position:", delta_x, delta_y)
    print('Move',x_direction,'by', delta_x)
    print('Move',y_direction,'by', delta_y)
    print('Move',z_direction,'by', delta_z)

#moveDrone(IMG_SHAPE,x,y,h,w)