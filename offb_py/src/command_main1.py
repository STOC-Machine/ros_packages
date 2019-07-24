#! /usr/bin/env python 

import rospy
from command_pub import command_pub

#initialize node
rospy.init_node('command_main1_node')

#creat object and publish
drone1_cmd = command_pub(1)
drone1_cmd.key_cmd()
