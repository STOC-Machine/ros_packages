#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 12:53:52 2019

@author: kota421
"""
import numpy as np
import cv2

def makeArray(image):
    array = np.zeros_like(image)
    return array

def labelArray(array,island,label):
    """
    array: zero_like array of shape == array.shape
    island: List of tuples where each tuple has two ints, representing a xy 
            coordinate of a point in an image.
    label: int
    """
    for (x,y) in island:
        array[x,y] = label
    return array
    
def BFS(x,y,image,isValidPixel):
    """
    Assumes:
        x,y: int, the start point of the flood fill
        isValidPixel: a function that given coordinates of the point tells you whether
                the point is valid or not
    Returns:
        island: list of tuples. Each tuple contains two ints, representing a xy 
            coordinate of a point.
    """
    island = []  # Represents an island, store valid pixels (valid: consecuitive AND 
              # check_validity == True)
    q = []
    q.append((x,y))
    island.append((x,y))
    while q:
        (x1,y1) = q.pop()
        if (isValidPixel(x1+1,y1,image)) and ((x1+1,y1) not in island):
            q.append((x1+1,y1))
            island.append((x1+1,y1))
        if (isValidPixel(x1-1,y1,image)) and ((x1-1,y1) not in island):
            q.append((x1-1,y1))
            island.append((x1-1,y1))
        if (isValidPixel(x1,y1+1,image)) and ((x1,y1+1) not in island):
            q.append((x1,y1+1))
            island.append((x1,y1+1))
        if (isValidPixel(x1,y1-1,image)) and ((x1,y1-1) not in island):
            q.append((x1,y1-1))
            island.append((x1,y1-1))
    return island

def getIslands(image,isValidPixel):
    """
    Assumes:
        image: cv2.image, gray
        isValidPixel: a function that, given coordinates of a point, tells you whether
                the point is valid or not
    Returns:
        islands: list of lists; each list contains xy coordinates of points as tuple
        label_arrays: np array of the same shape as image
    """ 
    islands = []
    master_label = makeArray(image)
    label_arrays = []
    label = 1 # Name for each island
    # At the first valid pixel encountered:
    for y in range(image.shape[1]):
        for x in range(image.shape[0]):
            # First, check if it already exists in islands,
            if master_label[x][y]: 
                pass
            # If it's not yet in islands, then start a floodfill from that pixel.
            elif isValidPixel(x,y,image):
                ## Flood-fill algorithm (BFS) 
                label_array = makeArray(image) 
                island = BFS(x,y,image,isValidPixel)
                ## Keeping track of valid pixels
                labelArray(label_array,island,label)
                labelArray(master_label,island,label)
                #print('Island Area[px]:',len(island))
                ## Adding a new island to islands
                islands.append(island)
                label_arrays.append(label_array)
                ## Changing island label for the next island
                label += 0
    return islands,label_arrays

def array2Coordinates(array):
    """
    Assumes:
        array:
            [
            [0 0 0 0 0 0 0 0...]
            [0 0 0 0 1 1 1 0...]
            [0 0 0 1 1 1 1 1...]
            [0 0 1 1 1 1 0 0...]
            ...
            ]
    Returns: 
        x, y: Two 1D np arrays: xy coordinates of a non-empty position 
            in the array
    """
    points = []
    for i in range(len(array)):
        for j in range(len(array[i])):
            if array[i][j]: # if it has a value (i.e. colored or white in image)
                points.append((i,j))
    points = np.array(points)
    ## Create x, y scatter poitns from POINTS
    y= points[:,0].reshape(-1,1)
    x= points[:,1].reshape(-1,1)
    return x,y

def maskingPolygonMaker(label_array):
    x,y = array2Coordinates(label_array)
    maxX, minX =max(x),min(x)
    maxY, minY =max(y),min(y)
    (h, w) = int((maxY-minY)[0]), int((maxX-minX)[0])
    polygon = np.array([
            [(minX,minY), (maxX,minY),(maxX,maxY),(minX,maxY)]
            ])
    mask = np.zeros_like(label_array)
    cv2.fillPoly(mask, polygon, 255)
    result = np.zeros([mask.shape[0],mask.shape[1],3]) # Conveting to a color image
    result[:,:,0] = mask
    result[:,:,1] = mask
    result[:,:,2] = mask
    result = result.astype(np.uint8)
    return result, (h,w)

def mask4ROI(image,isValidPixel,mode='biggest'):
    """
    Assumes:
        image: cv2.image gray.
        isValidPixel: a function that, given coordinates of a point, tells you whether
                the point is valid or not.
        mode: Method to choose which island to leave intact. 
            'biggest': leaves the biggest island , default
            'top':leaves the island on the top of the image
    Returns:
        i) If image has the target color:
            mask: numpy.array of shape == image.shape
            centerX, centerY: int, int. x-y coordinates of the center pixel of mask;
        ii) If image does not have the target color:
            None, None, None
    """ 
    islands,label_arrays = getIslands(image,isValidPixel)
    if not islands:
        return None, 0, 0,(0,0)
    
    if mode=='top':
        x,y = array2Coordinates(label_arrays[0])
        maxX, minX =max(x),min(x)
        maxY, minY =max(y),min(y)
        mask, (h,w) = maskingPolygonMaker(label_arrays[0])
        return mask, (maxX+minX)/2, (maxY+minY)/2,(h,w) 
    
    if mode=='biggest':
        island_areas = [] 
        for island in islands:
            island_areas.append(len(island))
        biggest = island_areas.index(max(island_areas)) 
        x,y = array2Coordinates(label_arrays[biggest])
        maxX, minX =max(x),min(x)
        maxY, minY =max(y),min(y)
        mask,(h,w)  = maskingPolygonMaker(label_arrays[biggest])
        return mask, int((maxX+minX)/2), int((maxY+minY)/2), (h,w) 

def region_of_interest(image,mask):
    """
    Assumes:
        image: color (width,height,3)
        mask: np.array (width,height,3) from the function mask4ROI 
    Returns:
        i) If mask is not empty:
            Bitwise and of image and mask
        ii) If mask is empty:
            image
        
    """
    # Is mask empty or not?
    try:
        mask.any()
        return cv2.bitwise_and(image, mask)
    except AttributeError:
        return image
    
    
    
        
## 
# =============================================================================
# def DFS(x, y, visited,image,isValidPixel):
#     """
#     Flood-fill algorithm in depth first search
#     """
#     n,m = image.shape[0],image.shape[1]
#     if (x >= n or y >= m):
#         return
#     if(x < 0 or y < 0):
#         return
#     if (x,y) in visited.keys():
#         return
#     visited[(x,y)] = True
#     if isValidPixel(x-1,y,image):
#         DFS(x-1, y, visited, image,isValidPixel)
#     if isValidPixel(x,y-1,image):
#         DFS(x, y-1, visited, image,isValidPixel)
#     if isValidPixel(x,y+1,image):
#         DFS(x, y+1, visited, image,isValidPixel)
#     if isValidPixel(x+1,y,image):
#         DFS(x+1, y, visited, image,isValidPixel)
# 
# =============================================================================
