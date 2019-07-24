#! /usr/bin/env python

import rospy
from sensor_pub import raspi_pub

#initialize node
rospy.init_node('sensor_main2_node')

drone2_sensor = raspi_pub(2)
drone2_sensor.publish()
