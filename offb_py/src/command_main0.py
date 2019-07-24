#! /usr/bin/env python 

import rospy
from command_pub import command_pub

#initialize node
rospy.init_node('command_main0_node')

#creat object and publish
drone0_cmd = command_pub(0)
drone0_cmd.key_cmd()
