#! /usr/bin/env python 

import rospy
from command_pub import command_pub

#initialize node
rospy.init_node('command_main0_node')

#ask for drone number
num = input("Enter number for drone (0 1 2 3): ")

#creat object and publish
drone_cmd = command_pub(num)
drone_cmd.key_cmd()
