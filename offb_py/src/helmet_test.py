#! /usr/bin/env python

from helmet_detection import test
import cv2

print "Start helmet test code"

#initialize capture
cap = cv2.VideoCapture(0)

#capture and detect object
for num in range(5):
    _, frame = cap.read()
    img_ROI, centerX, centerY, (h,w) = test(frame)
    print num, " x:", centerX, " y:", centerY


print "End helmet test code\n"

