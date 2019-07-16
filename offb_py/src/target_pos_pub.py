#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


class target_pos_pub(object):
    def __init__(self, num):
        #topics
        self.position_pub = rospy.Publisher('/uav' + str(num) + 
                '/mavros/target_pos', PoseStamped, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/uav' + str(num) + 
                '/mavros/command', Float32MultiArray, self.cmd_cb)
        self.pos_sub = rospy.Subscriber('uav' + str(num) + 
                'mavros/setpoint_position/local', PoseStamped, self.current_pos_cb)
        
        #variables
        self.pos = PoseStamped()
        self.current_pos = PoseStamped()
        self.cmd_val = "Stop"
        self.buff_pos = PoseStamped()
        self.rate = rospy.Rate(2)
        self.ctrl_c = False

        #shutdown
        rospy.on_shutdown(self.shutdownhook)


    def current_pos_cb(self, msg):
        self.current_pose = msg


    def cmd_cb(self, msg):
        self.cmd_val = msg.data[0]
        self.buff_pos = self.current_pos
        if self.cmd_val >= 1 and self.cmd_val <= 3:
            self.buff_pos.pose.position.x += msg.data[1]
        elif self.cmd_val == 4:
            self.buff_pos.pose.orientation.y += msg.data[1]
        elif self.cmd_val == 5:
            self.buff_pos.pose.position.x += msg.data[1]
            self.buff_pos.pose.position.y += msg.data[2]


    def shutdownhook(self):
        self.ctrl_c = True
        print "shuting down target_pos_pub"


    def pub_target(self):
        while not self.ctrl_c:
            #publish target position
            self.position_pub.publish(self.buff_pos)
            self.rate.sleep()




if __name__ == "__main__":
    rospy.init_node('target_pos_pub_node')

    target_publisher = target_pos_pub(0)
    target_publisher.pub_target()




