#! /usr/bin/env python

import rospy
from sensor_pub import raspi_pub

#initialize node
rospy.init_node('sensor_main1_node')

drone1_sensor = raspi_pub(1)
drone1_sensor.publish()
