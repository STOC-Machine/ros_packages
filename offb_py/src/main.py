#! /usr/bin/env python

from multi_drone import multi_drone
import threading
import rospy

if __name__ == '__main__':
    rospy.init_node('main_node')
    
    #create threads
    drone0 = multi_drone(0)
    drone_thread0 = threading.Thread(target=drone0.fly())

    drone1 = multi_drone(1)
    drone_thread1 = threading.Thread(target=drone1.fly())


    #wait for drones to exit
    drone_thread0.join()
    drone_thread1.join()
