#! /usr/bin/env python

import rospy
from sensor_pub import raspi_pub

#initialize node
rospy.init_node('sensor_main3_node')

drone3_sensor = raspi_pub(3)
drone3_sensor.publish()
