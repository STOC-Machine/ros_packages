#! /usr/bin/env python

from multi_drone import multi_drone
import threading
import rospy

if __name__ == '__main__':
    rospy.init_node('main_node')
    

    #create drone objects
    drone0 = multi_drone(0)
    drone1 = multi_drone(1)


    #shutdown hook function
    def shutdownhook():
        global drone0
        global drone1

        rospy.loginfo("shutdown time")

        drone0.shutdownhook()
        drone1.shutdownhook()


    #create flying functions
    def fly_0():
        drone0.fly()

    def fly_1():
        drone1.fly()


    #create threads
    drone_thread0 = threading.Thread(target=fly_0, name='drone_thread0')
    drone_thread1 = threading.Thread(target=fly_1, name='drone_thread1')


    #start threads
    drone_thread0.start()
    drone_thread1.start()


    #rospy shutdown
    rospy.on_shutdown(shutdownhook)


    #wait for drones to exit
    drone_thread0.join()
    drone_thread1.join()
