#! /usr/bin/env python 

import rospy
from command_pub import command_pub

#initialize node
rospy.init_node('command_main3_node')

#creat object and publish
drone3_cmd = command_pub(3)
drone3_cmd.key_cmd()
