#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import threading
import math



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
        self.obstacle = False
        self.height = 2.5

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
                    self.forward(1.0)
                elif direction == 1:
                    self.forward(-1.0)
                elif direction == 2:
                    self.up(0.5)
                elif direction == 3:
                    self.up(-0.5)
                elif direction == 4:
                    self.right(1.0)
                elif direction == 5:
                    self.right(-1.0)
            elif action == 1:
                if direction == 0:
                    self.turn_ccw()
                elif direction == 1:
                    self.turn_cw()
            elif action == 2:
                self.stop()
            elif action == 4:
                #todo
                notta = 1
            elif action == 5:
                self.set_mode("AUTO.LAND")

    def sensor_cb(self, sensor_msg):
        if sensor_msg.data[3] < 50 and self.obstacle:
            self.object_flag = True
            self.forward(-1)
            print "Obstacle Detected!!!"
        else:
            self.object_flag = False
            if self.helmet_flag:
                print "helmet x: ", sensor_msg.data[1]
                if sensor_msg.data[1] > 1:
                    xh = sensor_msg.data[1] - 100
                    yh = sensor_msg.data[2] - 50
                    self.right(0.02 * xh)
                    self.forward(-0.03 * yh)


    def wait_for_connection(self):
        while not self.ctrl_c and not self.state.connected:
            self.rate.sleep()

    def wait(self, time_sec):
        print "wait start"
        for _ in range(int(time_sec * 20)):
            self.rate.sleep()
        print "wait end"

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

        # test helmet detection
        self.wait(5)
        self.obstacle = True
        #self.helmet_flag = True
        #self.right(1)
        #self.wait(2)
        #self.forward(1)

        # land
        input("land?")
        self.set_mode("AUTO.LAND")

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
        print "forward start"
        #self.position_send_obj = self.position_obj
        self.position_send_obj.pose.position.x = self.position_obj.pose.position.x
        self.position_send_obj.pose.position.y = self.position_obj.pose.position.y
        # calculate x and y distance
        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        x_distance = distance * ((qw*qw) - (qz*qz)) 
        y_distance = distance * 2 * qz * qw
        # add x and y distance
        self.position_send_obj.pose.position.x += x_distance
        self.position_send_obj.pose.position.y += y_distance
        self.position_send_obj.pose.position.z = self.height
        print "forward end"

    def right(self, distance):
        # self.position_send_obj = self.position_obj
        # calculate x and y distance
        self.position_send_obj.pose.position.x = self.position_obj.pose.position.x
        self.position_send_obj.pose.position.y = self.position_obj.pose.position.y
        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        x_distance = distance * (2 * qw * qz) 
        y_distance = (-1.0) * distance * ((qw * qw) - (qz * qz))
        # add x and y distance
        self.position_send_obj.pose.position.x += x_distance
        self.position_send_obj.pose.position.y += y_distance

    def turn_ccw(self):
        #self.position_send_obj = self.position_obj
        self.position_send_obj.pose.position.x = self.position_obj.pose.position.x
        self.position_send_obj.pose.position.y = self.position_obj.pose.position.y

        # calculate quarternion
        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        if qz >= 0.0:
            new_rad = 2 * math.acos(qw)
        elif qz <= 0.0:
            new_rad = -2 * math.acos(qw)

        if new_rad < 0.0:
            new_rad += 6.28

        # set rotation angle
        if (new_rad >= -0.8 and new_rad <= 0.8) or (new_rad >= 5.4 and new_rad <= 7.1) or (new_rad >= -5.4 and new_rad <= -7.1):
            #forward to left
            self.position_send_obj.pose.orientation.z = 0.707
            self.position_send_obj.pose.orientation.w = 0.707
        elif (new_rad >= 0.7 and new_rad <= 2.4) or (new_rad <= -3.9 and new_rad >= -5.5):
            #left to back
            self.position_send_obj.pose.orientation.z = 1
            self.position_send_obj.pose.orientation.w = 0
        elif (new_rad >= 2.3 and new_rad <= 4.0) or (new_rad <= -2.3 and new_rad >= -4.0):
            #back to right
            self.position_send_obj.pose.orientation.z = -0.707
            self.position_send_obj.pose.orientation.w = 0.707
 
        elif (new_rad >= 3.9 and new_rad <= 5.5) or (new_rad <= -0.7 and new_rad >= -2.4):
            #right to front
            self.position_send_obj.pose.orientation.z = 0
            self.position_send_obj.pose.orientation.w = 1


    def turn_cw(self):
        #self.position_send_obj = self.position_obj
        self.position_send_obj.pose.position.x = self.position_obj.pose.position.x
        self.position_send_obj.pose.position.y = self.position_obj.pose.position.y

        # calculate quarternion
        qz = self.position_obj.pose.orientation.z
        qw = self.position_obj.pose.orientation.w
        if qz >= 0.0:
            new_rad = 2 * math.acos(qw)
        elif qz <= 0.0:
            new_rad = -2 * math.acos(qw)

        if new_rad < 0.0:
            new_rad += 6.28

        # set rotation angle
        if (new_rad >= -0.8 and new_rad <= 0.8) or (new_rad >= 5.4 and new_rad <= 7.1) or (new_rad >= -5.4 and new_rad <= -7.1):
            #forward to right
            self.position_send_obj.pose.orientation.z = -0.707
            self.position_send_obj.pose.orientation.w = 0.707
        elif (new_rad >= 0.7 and new_rad <= 2.4) or (new_rad <= -3.9 and new_rad >= -5.5):
            #right to back
            self.position_send_obj.pose.orientation.z = -1
            self.position_send_obj.pose.orientation.w = 0
        elif (new_rad >= 2.3 and new_rad <= 4.0) or (new_rad <= -2.3 and new_rad >= -4.0):
            #back to left
            self.position_send_obj.pose.orientation.z = 0.707
            self.position_send_obj.pose.orientation.w = 0.707
 
        elif (new_rad >= 3.9 and new_rad <= 5.5) or (new_rad <= -0.7 and new_rad >= -2.4):
            #right to front
            self.position_send_obj.pose.orientation.z = 0
            self.position_send_obj.pose.orientation.w = 1
 
 

    def up(self, distance):
        #self.position_send_obj = self.position_obj
        self.position_send_obj.pose.position.z += distance

    def stop(self):
        #print "stop start"
        #for _ in range(5):
        self.position_send_obj = self.position_obj
        #self.rate.sleep()
        #print "stop end"

    def takeoff(self):
        self.position_send_obj = self.position_obj
        #self.xpos = self.position_obj.pose.position.x
        #self.ypos = self.position_obj.pose.position.y
        for _ in range(10):
            self.position_send_obj.pose.position.z = self.height

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
