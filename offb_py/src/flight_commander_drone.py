#! /usr/bin/env python

import rospy
from multi_drone import multi_drone

#initialze ros node
rospy.init_node('flight_commander_drone_node')
rospy.loginfo('initialized flight_commander_drone_node')

#ask for drone number
num = input("Enter number for drone (0 1 2 3): ")

#create multi_drone object
drone = multi_drone(num)

#call flight_commander() function
drone.flight_commander()
