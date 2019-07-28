#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

class cmd_pub(object):
    def __init__(self):
        #topics
        self.command_pub_0 = rospy.Publisher('uav0/mavros/command', 
                Int32MultiArray, queue_size=1)
        self.command_pub_1 = rospy.Publisher('uav1/mavros/command', 
                Int32MultiArray, queue_size=1)
        self.command_pub_2 = rospy.Publisher('uav2/mavros/command', 
                Int32MultiArray, queue_size=1)
        self.command_pub_3 = rospy.Publisher('uav3/mavros/command', 
                Int32MultiArray, queue_size=1)

        #variables
        self.cmd = Int32MultiArray()
        self.rate = rospy.Rate(20)
        self.ctrl_c = False

        #wait for initialization
        for _ in range(20):
            self.rate.sleep()

        #shutdown
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True

    def publish(self, msg):
        # set command msg
        drone_num, action, direction = msg
        self.cmd.data = [action, direction]

        # publish to specific drone
        if drone_num == 0:
            self.command_pub_0.publish(self.cmd)
        elif drone_num == 1:
            self.command_pub_1.publish(self.cmd)
        elif drone_num == 2:
            self.command_pub_2.publish(self.cmd)
        elif drone_num == 3:
            self.command_pub_3.publish(self.cmd)
        elif drone_num == 4:
            self.command_pub_0.publish(self.cmd)
            self.command_pub_1.publish(self.cmd)
            self.command_pub_2.publish(self.cmd)
            self.command_pub_3.publish(self.cmd)



if __name__ == '__main__':
    rospy.init_node('cmd_pub_node')

    # test publish to all drones
    rate = rospy.Rate(1)
    print "begin tests"
    cmd_obj = cmd_pub()
    print "created object"
    cmd_obj.publish(0,0,0)
    print "published to 0"
    rate.sleep()
    cmd_obj.publish(1,1,1)
    print "published to 1"
    rate.sleep()
    cmd_obj.publish(-1,3,0)
    print "published to -1"
    rospy.spin()
    

