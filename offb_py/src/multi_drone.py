#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import threading

class multi_drone(object):
    def __init__(self, num):
        if num < 0 or num > 80:
            print "Error: initializing ", num, " drone. Bad number"
            self.shutdown_hook()
        self.state_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/state', State, self.state_cb)
        self.pose_pub = rospy.Publisher('/uav' + str(num) + '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.arming_client = rospy.ServiceProxy('/uav' + str(num) + '/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/uav' + str(num) + '/mavros/set_mode', SetMode)

        self.pose_obj = PoseStamped()
        self.offb_srv_msg = SetModeRequest()
        self.arming_srv_msg = CommandBoolRequest()
        self.state = State()
        self.state.connected = False
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(20)

    def shutdown_hook(self):
        self.set_pose(0,0,0)
        for _iter in range(80):
            self.pose_pub.publish(self.pose_obj)
            self.rate.sleep()

    def state_cb(self, state_msg):
        self.state = state_msg

    def wait_for_connection(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()

    def set_pose(self, x, y, z):
        self.pose_obj.pose.position.x = x
        self.pose_obj.pose.position.y = y
        self.pose_obj.pose.position.z = z

    def fly(self):
        self.wait_for_connection()
        self.set_pose(0,0,2)
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

            #publish position
            self.pose_pub.publish(self.pose_obj)

            #sleep
            self.rate.sleep()



#run code if this is the main file
if __name__ == "__main__":
    rospy.init_node('multi_drone_node')
    rospy.loginfo("initialized multi_drone_node")
    
    drone0 = multi_drone(0)
    drone1 = multi_drone(1)

    def fly_0():
        drone0.fly()

    def fly_1():
        drone1.fly()

    t0 = threading.Thread(target=fly_0, name='t0')
    t1 = threading.Thread(target=fly_1, name='t1')

    t0.start()
    t1.start()

    t0.join()
    t1.join()

    #drone0.fly()
    #drone1.fly()
