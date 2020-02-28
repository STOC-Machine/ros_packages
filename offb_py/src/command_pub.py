#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

class command_pub(object):
    def __init__(self, num):
        #topics
        self.command_pub = rospy.Publisher('uav' + str(num) + '/mavros/command', Int32MultiArray, queue_size=1)

        #variables
        self.cmd = Int32MultiArray()
        self.ctrl_c = False

        #shutdown
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        self.ctrl_c = True
        self.cmd.data[0] = 0
        self.command_pub.publish(self.cmd)

    def key_cmd(self):
        while not self.ctrl_c:
            num = input("Cmd (0-fly 1-turn 2-stop 3-save 4-special 5-land): ")
            direction = input("direction: 0-forward 1-back 2-up/turn forward 3-down 4-right 5-left")
            self.cmd.data = [num, direction]

            #publish cmd
            self.command_pub.publish(self.cmd)



if __name__ == '__main__':
    rospy.init_node('command_pub_node')

    #command_pub object and key commands
    cmd_obj = command_pub(0)
    cmd_obj.key_cmd()

