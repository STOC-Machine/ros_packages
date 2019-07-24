#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import threading

class multi_drone(object):
    def __init__(self, num):
        if num < 0 or num > 80:
            print "Error: initializing ", num, " drone. Bad number"
            self.shutdown_hook()

        #topics
        self.state_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/state', 
                State, self.state_cb)
        self.pose_pub = rospy.Publisher('/uav' + str(num) + 
                '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.arming_client = rospy.ServiceProxy('/uav' + str(num) + 
                '/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/uav' + str(num) + 
                '/mavros/set_mode', SetMode)
        self.command_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/command',
                Float32MultiArray, self.cmd_cb)
        self.sensor_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/sensor',
                Float32MultiArray, self.sensor_cb)

        #variables
        self.pose_obj = PoseStamped()
        self.offb_srv_msg = SetModeRequest()
        self.arming_srv_msg = CommandBoolRequest()
        self.state = State()
        self.state.connected = False
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(20)
        self.helmet_flag = False
        self.object_flag = False
        self.ctrl_c = False

        #shutdown
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.set_pose(0,0,0)
        for _iter in range(80):
            self.pose_pub.publish(self.pose_obj)
            self.rate.sleep()
        self.ctrl_c = True

    def state_cb(self, state_msg):
        self.state = state_msg

    def cmd_cb(self, msg):
        if msg.data[0] == 6:
            self.helmet_flag = True
        else:
            self.helmet_flag = False
            if not self.object_flag:
                self.pose_obj.pose.position.x += msg.data[1]
                self.pose_obj.pose.orientation.z = msg.data[2]
                self.pose_obj.pose.orientation.w = msg.data[3]

    def sensor_cb(self, sensor_msg):
        if sensor_msg.data[3] < 30:
            self.object_flag = True
        else:
            self.object_flag = False
            if self.helmet_flag:
                self.pose_obj.pose.position.x += sensor_msg.data[0]
                self.pose_obj.pose.position.y += sensor_msg.data[0]


    def wait_for_connection(self):
        while not self.ctrl_c and not self.state.connected:
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
        while not self.ctrl_c:
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


    def flight_commander(self):
        position_thread = threading.Thread(target=self.publish_pose)
        position_thread.start()
        while not self.ctrl_c:
            cmd_num = input("1-offb 2-arm 3-takeoff 4-front 5-back 6-right 7-left 8-disarm 9-land 0-exit")
            if cmd_num == 1:
                self.set_mode("OFFBOARD")
            elif cmd_num == 2:
                self.arm_drone(True)
            elif cmd_num == 3:
                self.set_pose(0,0,2)
            elif cmd_num == 4:
                self.set_pose(1,0,2)
            elif cmd_num == 5:
                self.set_pose(-1,0,2)
            elif cmd_num == 6:
                self.set_pose(0,1,2)
            elif cmd_num == 7:
                self.set_pose(0,-1,2)
            elif cmd_num == 8:
                self.arm_drone(False)
            elif cmd_num == 9:
                self.set_mode("AUTO.LAND")
            elif cmd_num == 0:
                break

        self.set_mode("AUTO.LAND")
        self.shutdownhook()

    def set_mode(self, mode_name):
        self.offb_srv_msg.custom_mode = mode_name
        while not self.ctrl_c:
            if self.mode_client.call(self.offb_srv_msg).mode_sent:
                rospy.loginfo("set to offboard mode!")
                break
            for _ in range(40):
                self.rate.sleep()

    def arm_drone(self, arm_bool):
        self.arming_srv_msg.value = arm_bool
        while not self.ctrl_c:
            if self.arming_client.call(self.arming_srv_msg).success:
                return_msg = "armed: " + str(arm_bool)
                rospy.loginfo(return_msg)
                break
            for _ in range(40):
                self.rate.sleep(40)

    def publish_pose(self):
        while not self.ctrl_c:
            self.pose_pub.publish(self.pose_obj)
            self.rate.sleep()


#run code if this is the main file
if __name__ == "__main__":
    rospy.init_node('multi_drone_node')
    rospy.loginfo("initialized multi_drone_node")
    
    drone0 = multi_drone(0)

    drone0.flight_commander()

"""
    t0.start()
    t1.start()

    t0.join()
    t1.join()

    #drone0.fly()
    #drone1.fly()
"""
