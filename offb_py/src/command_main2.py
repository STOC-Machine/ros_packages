#! /usr/bin/env python 

import rospy
from command_pub import command_pub

#initialize node
rospy.init_node('command_main2_node')

#creat object and publish
drone2_cmd = command_pub(2)
drone2_cmd.key_cmd()
