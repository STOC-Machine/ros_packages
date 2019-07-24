#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class hover_drone(object):
    def __init__(self):
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.pose_obj = PoseStamped()
        self.offb_srv_msg = SetModeRequest()
        self.arming_srv_msg = CommandBoolRequest()
        self.state = State()
        self.state.connected = False
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(20)

    def state_cb(self, state_msg):
        self.state = state_msg

    def wait_for_connection(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()

    def set_pose(self, x, y, z, ox=0, oy=0, oz=0, ow=0):
        self.pose_obj.pose.position.x = x
        self.pose_obj.pose.position.y = y
        self.pose_obj.pose.position.z = z
        self.pose_obj.pose.orientation.x = ox
        self.pose_obj.pose.orientation.y = oy
        self.pose_obj.pose.orientation.z = oz
        self.pose_obj.pose.orientation.w = ow


    def fly(self):
        self.wait_for_connection()
        self.set_pose(0,0,2,0,0,0)
        for _iterations in range(100):
            self.pose_pub.publish(self.pose_obj)
            self.rate.sleep()

        #set up service messages for arming and offb mode
        self.offb_srv_msg.custom_mode = "OFFBOARD"
        self.arming_srv_msg.value = True

        #while loop for arming, offb mode, and publish position
        ctrl_c = False
        while not ctrl_c:
            #set to offb mode
            if self.state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_time > rospy.Duration(5.0) ):
                if self.mode_client.call(self.offb_srv_msg).mode_sent:
                    rospy.loginfo("set to offboard mode!")
                self.last_time = rospy.Time.now()
            else:
                if not self.state.armed and (rospy.Time.now() - self.last_time > rospy.Duration(5.0) ):
                    if self.arming_client.call(self.arming_srv_msg).success:
                        rospy.loginfo("vehicle armed!!")
                    self.last_time = rospy.Time.now()

            #test flight directions
            if rospy.Time.now() - self.last_time > rospy.Duration(6.0):
                self.set_pose(0,0,2, 0.0,0.0,0.383,-0.924)

            #publish position
            self.pose_pub.publish(self.pose_obj)

            #sleep
            self.rate.sleep()



#run code if this is the main file
if __name__ == "__main__":
    rospy.init_node('hover_test_node')
    rospy.loginfo("initialized hover_test_node")
    
    drone = hover_drone()
    drone.fly()
