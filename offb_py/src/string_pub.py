#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import socket

class string_pub(object):
    def __init__(self, num):
        #topics
        self.command_pub_0 = rospy.Publisher('uav0/mavros/command', 
                Float32MultiArray, queue_size=1)
        self.command_pub_1 = rospy.Publisher('uav1/mavros/command', 
                Float32MultiArray, queue_size=1)
        self.command_pub_2 = rospy.Publisher('uav2/mavros/command', 
                Float32MultiArray, queue_size=1)
        self.command_pub_3 = rospy.Publisher('uav3/mavros/command', 
                Float32MultiArray, queue_size=1)

        #variables
        self.cmd = Float32MultiArray()
        self.ctrl_c = False

        #shutdown
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        self.ctrl_c = True
        self.cmd.data[0] = 0
        self.command_pub.publish(self.cmd)

    def key_cmd(self):
        while not self.ctrl_c:
            num = input("Cmd (1-helmet 4-forward 5-back 6-right 7-left): ")
            self.cmd.data = [num, 0, 0, 1]
            if num == -1:
                break
            elif num == 4:
                self.cmd.data[1] = 1
            elif num == 5:
                self.cmd.data[1] = -1
            elif num == 6:
                self.cmd.data[2] = 0.707
                self.cmd.data[3] = 0.707
            elif num == 7:
                self.cmd.data[2] = -0.707
                self.cmd.data[3] = 0.707

            #publish cmd
            self.command_pub.publish(self.cmd)



if __name__ == '__main__':
    rospy.init_node('command_pub_node')

    #command_pub object and key commands
    cmd_obj = command_pub(0)
    cmd_obj.key_cmd()

