#! /usr/bin/env python

import rospy
from sensor_pub import raspi_pub

#initialize node
rospy.init_node('sensor_main0_node')

drone0_sensor = raspi_pub(0)
drone0_sensor.publish()
