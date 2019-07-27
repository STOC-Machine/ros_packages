#! /usr/bin/env python

import rospy
import threading
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist



########        callback functions      ########
# shut down function
def shutdownhook():
    global ctrl_c
    ctrl_c = True
    #print "shutdown-", ctrl_c

# state subscriber callback
def state_cb(msg):
    global state_msg
    state_msg = msg

# local position callback
def pose_cb(msg):
    global pose_msg
    pose_msg = msg

# publish velocity commands
def velocity_publisher():
    global velocity_pub
    global velocity_msg
    global ctrl_c
    global rate
    while not ctrl_c:
        velocity_pub.publish(velocity_msg)
        rate.sleep()

# wait function
def wait(time):
    global rate
    for _ in range(int(time*20)):
        rate.sleep()

# set mode function
def set_mode(mode_name):
    global set_mode_msg
    global ctrl_c
    set_mode_msg.custom_mode = mode_name
    while not ctrl_c:
        if set_mode_srv.call(set_mode_msg).mode_sent:
            print "set to ", mode_name, "!"
            break
        wait(2)

# arming function
def arm(arm_bool):
    global ctrl_c
    global arming_msg
    arming_msg.value = arm_bool
    while not ctrl_c:
        if arming_srv.call(arming_msg).success:
            print "drone armed: ", arm_bool
            break
        wait(2)

# forward function
def forward(distance):
    speed = 0.75
    global ctrl_c
    global rate
    global velocity_msg
    global pose_msg
    qz = pose_msg.pose.orientation.z
    qw = pose_msg.pose.orientation.w
    x_speed = speed * ((qw*qw) - (qz*qz)) 
    y_speed = speed * 2 * qz * qw
    velocity_msg.linear.x = x_speed
    velocity_msg.linear.y = y_speed
    wait( (distance*1.0) / speed )
    velocity_msg.linear.x = 0.0
    velocity_msg.linear.y = 0.0

# turn right function
def turn_right(radians):
    ang_speed = 0.5
    global ctrl_c
    global rate
    global velocity_msg
    global pose_msg
    velocity_msg.linear.x = 0.0
    velocity_msg.linear.y = 0.0
    velocity_msg.angular.z = ang_speed
    wait( (radians * 1.0) / (ang_speed) )
    velocity_msg.angular.z = 0.0



########        initialize ros      ########
rospy.init_node('test_drone_node')

# state subscriber
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
state_msg = State()

# local position subscriber
pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
pose_msg = PoseStamped()

# velocity publisher
velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', 
        Twist, queue_size=1)
velocity_msg = Twist()

# set_mode service
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
set_mode_msg = SetModeRequest()

# arm service
arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
arming_msg = CommandBoolRequest()

# rest of initialization
ctrl_c = False
rospy.on_shutdown(shutdownhook)
rate = rospy.Rate(20)



########        set up flight       ########
# wait for connection to drone
while not ctrl_c and not state_msg.connected:
    rate.sleep()

# publish velocity thread
velocity_thread = threading.Thread(target=velocity_publisher)
velocity_thread.daemon = True



########        test next       ########
print "start test"

# start publishing velocity
velocity_thread.start()

# set to offboard mode
wait(1)
set_mode("OFFBOARD")

# arm vehicle
wait(1)
arm(True)

# find local position
print "z position: ", pose_msg.pose.position.z

# z velocity
velocity_msg.linear.z = 1
while not ctrl_c:
    if pose_msg.pose.position.z >= 2:
        break
    rate.sleep()
velocity_msg.linear.z = 0

# fly around
wait(1)
turn_right(4.6)
wait(1)
forward(2)

# wait
rospy.spin()

print "done"
















