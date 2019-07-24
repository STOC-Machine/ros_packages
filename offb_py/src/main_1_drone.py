#! /usr/bin/env python

from multi_drone import multi_drone
import rospy

if __name__ == '__main__':
    rospy.init_node('main_1_drone_node')
    print "initialized main_node"
    

    #create drone objects
    drone0 = multi_drone(0)
    print "created multi_drone object"


    #shutdown hook function
    def shutdownhook():
        global drone0

        rospy.loginfo("shutdown time")

        drone0.shutdownhook()


    #start flying
    drone0.fly()
    print "start flying"


    #rospy shutdown
    rospy.on_shutdown(shutdownhook)
    print "wating for rospy.shutdown"

