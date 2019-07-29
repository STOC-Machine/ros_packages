#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import threading



##################################################
########        multi_drone class       ##########
##################################################

class multi_drone(object):

    ########        initialization function     ########
    def __init__(self, num):
        if num < 0 or num > 80:
            print "Error: initializing ", num, " drone. Bad number"
            self.shutdown_hook()

        #topics
        self.state_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/state', 
                State, self.state_cb)
        self.position_sub = rospy.Subscriber('/uav' + str(num) + 
                '/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.velocity_pub = rospy.Publisher('/uav' + str(num) + 
                '/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.position_pub = rospy.Publisher('uav' + str(num) +
                '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.arming_client = rospy.ServiceProxy('/uav' + str(num) + 
                '/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/uav' + str(num) + 
                '/mavros/set_mode', SetMode)
        self.command_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/command',
                Int32MultiArray, self.cmd_cb)
        self.sensor_sub = rospy.Subscriber('/uav' + str(num) + '/mavros/sensor',
                Float32MultiArray, self.sensor_cb)

        #variables
        self.velocity_obj = Twist()
        self.position_obj = PoseStamped()
        self.position_send_obj = PoseStamped()
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
        self.ctrl_c = True

    def state_cb(self, state_msg):
        self.state = state_msg

    def pose_cb(self, msg):
        self.position_obj = msg

    def cmd_cb(self, msg):
        action = msg.data[0]
        direction = msg.data[1]
        if action == 3:
            self.helmet_flag = True
        else:
            self.helmet_flag = False
            if action == 0:
                if direction == 0:
                    self.forward(1)
                elif direction == 1:
                    self.back(1)
                elif direction == 2:
                    self.up(1)
                elif direction == 3:
                    self.down(1)
            elif action == 1:
                if direction == 0:
                    self.turn_ccw(1.55)
                elif direction == 1:
                    self.turn_cw(1.55)
            elif action == 2:
                self.stop()
            elif action == 5:
                self.set_mode("AUTO.LAND")

    def sensor_cb(self, sensor_msg):
        if sensor_msg.data[3] < 100:
            self.object_flag = True
            print "Obstacle Detected!!!"
        else:
            self.object_flag = False
            if self.helmet_flag:
                # TODO
                num_test = 1


    def wait_for_connection(self):
        while not self.ctrl_c and not self.state.connected:
            self.rate.sleep()

    def wait(self, time_sec):
        for _ in range(int(time_sec * 20)):
            self.rate.sleep()

    def fly(self):
        #wait for vehicle to connect
        self.wait_for_connection()

        #start publishing position
        position_thread = threading.Thread(target=self.publish_position)
        position_thread.daemon = True
        position_thread.start()

        #wait for a few messages to be sent
        self.wait(1)

        #change mode to offboard mode
        input("set to offboard?")
        self.set_mode("OFFBOARD")
        self.wait(1)
        
        #arm and take off
        input("arm?")
        self.arm_drone(True)
        self.takeoff()

        # forward
        #input("forward?")
        #self.forward(1)

        #wait for close
        rospy.spin()


    """def flight_commander(self):
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
        self.shutdownhook()"""

    def set_mode(self, mode_name):
        self.offb_srv_msg.custom_mode = mode_name
        while not self.ctrl_c:
            if self.mode_client.call(self.offb_srv_msg).mode_sent:
                return_msg = "set to " + mode_name + " mode!"
                rospy.loginfo(return_msg)
                break
            self.wait(2)

    def arm_drone(self, arm_bool):
        self.arming_srv_msg.value = arm_bool
        while not self.ctrl_c:
            if self.arming_client.call(self.arming_srv_msg).success:
                return_msg = "armed: " + str(arm_bool)
                rospy.loginfo(return_msg)
                break
            self.wait(2)

    def publish_velocity(self):
        while not self.ctrl_c:
            self.velocity_pub.publish(self.velocity_obj)
            self.rate.sleep()

    def publish_position(self):
        while not self.ctrl_c:
            self.position_pub.publish(self.position_send_obj)
            self.rate.sleep()

    ########        control functions       ########

    def forward(self, distance):
        self.position_send_obj = self.position_obj

        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        x_distance = distance * ((qw*qw) - (qz*qz)) 
        y_distance = distance * 2 * qz * qw
        self.velocity_obj.linear.x = x_speed
        self.velocity_obj.linear.y = y_speed
        self.wait( (distance*1.0) / speed )
        self.velocity_obj.linear.x = 0.0
        self.velocity_obj.linear.y = 0.0

    def back(self, distance):
        speed = 0.75
        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        x_speed = (-1.0) * speed * ((qw*qw) - (qz*qz)) 
        y_speed = (-1.0) * speed * 2 * qz * qw
        self.velocity_obj.linear.x = x_speed
        self.velocity_obj.linear.y = y_speed
        self.wait( (distance*1.0) / speed )
        self.velocity_obj.linear.x = 0.0
        self.velocity_obj.linear.y = 0.0

    def turn_cw(self, radians):
        ang_speed = 1.0
        self.velocity_obj.linear.x = 0.0
        self.velocity_obj.linear.y = 0.0
        self.velocity_obj.angular.z = (-1.0) * ang_speed
        self.wait( (radians * 1.0) / (ang_speed) )
        self.velocity_obj.angular.z = 0.0

    def turn_ccw(self, radians):
        ang_speed = 1.0
        self.velocity_obj.linear.x = 0.0
        self.velocity_obj.linear.y = 0.0
        self.velocity_obj.angular.z = ang_speed
        self.wait( (radians * 1.0) / (ang_speed) )
        self.velocity_obj.angular.z = 0.0

    def up(self, distance):
        speed = 0.5
        self.velocity_obj.linear.z = speed
        self.wait( (distance*1.0) / speed )
        self.velocity_obj.linear.z = 0.0

    def down(self, distance):
        speed = 0.5
        self.velocity_obj.linear.z = (-1.0) * speed
        self.wait( (distance*1.0) / speed )
        self.velocity_obj.linear.z = 0.0

    def stop(self):
        for _ in range(5):
            self.velocity_obj.linear.x = 0.0
            self.velocity_obj.linear.y = 0.0
            self.velocity_obj.linear.z = 0.0
            self.velocity_obj.angular.x = 0.0
            self.velocity_obj.angular.y = 0.0
            self.velocity_obj.angular.z = 0.0
            self.rate.sleep()

    def takeoff(self):
        self.position_send_obj = self.position_obj
        for _ in range(10):
            self.position_send_obj.pose.position.z = 3
            self.rate.sleep()






############################################
########         main program       ########
############################################

if __name__ == "__main__":
    rospy.init_node('multi_drone_node')
    rospy.loginfo("initialized multi_drone_node")
    
    drone0 = multi_drone(0)

    drone0.fly()

"""
    t0.start()
    t1.start()

    t0.join()
    t1.join()

    #drone0.fly()
    #drone1.fly()
"""
