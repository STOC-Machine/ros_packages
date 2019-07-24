#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

class command_pub(object):
    def __init__(self, num):
        #topics
        self.command_pub = rospy.Publisher('uav' + str(num) + '/mavros/command',
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
            num = input("Enter number for command: ")
            self.cmd.data = [num, 0, 0, 1]
            if num == -1:
                break
            elif num == 1:
                self.cmd.data[1] = 1
            elif num == 3:
                self.cmd.data[1] = -1
            elif num == 4:
                self.cmd.data[2] = 0.707
                self.cmd.data[3] = 0.707
            elif num == 5:
                self.cmd.data[2] = -0.707
                self.cmd.data[3] = 0.707

            #publish cmd
            self.command_pub.publish(self.cmd)



if __name__ == '__main__':
    rospy.init_node('command_pub_node')

    #command_pub object and key commands
    cmd_obj = command_pub(0)
    cmd_obj.key_cmd()

