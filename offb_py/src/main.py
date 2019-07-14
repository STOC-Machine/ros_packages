#! /usr/bin/env python

from helmet_detection import 
from hover_test import hover_drone
from rangeFinder import range_finder
import rospy

if __name__ == '__main__':
   rospy.init_node('main_node')

   drone = hover_drone()
   
    #test ultrasonic range sensor
   range_sensor = range_finder()
   range_sensor.loop()

   drone.fly()
