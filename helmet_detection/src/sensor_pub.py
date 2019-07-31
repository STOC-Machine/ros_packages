#! /usr/bin/env python

import rospy
import io
import socket
import struct
import time
import cv2
import numpy
from PIL import Image
from helmet_detection import test
from std_msgs.msg import Float32MultiArray, Bool


class raspi_pub(object):
    def __init__(self, num):
        #topic and messages
        self.drone_sensors_pub = rospy.Publisher('/uav' + str(num) + 
                '/mavros/sensor', Float32MultiArray, queue_size=1) 
        self.sensor_data = Float32MultiArray()
        self.sensor_data.data = [0,0,0,0]

        #tcp connection - socket listening on all interfaces:port
        self.server_socket = socket.socket()
        self.server_socket.bind(('0.0.0.0', 8000 + num))
        self.server_socket.listen(0)

        # Accept a single connection and make a file-like object out of it
        self.connection = self.server_socket.accept()[0].makefile('rb')

        # udp connection for distance sensor
        self.distance_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.distance_socket.bind(('0.0.0.0', 8010 + num))

        ### qr code ###
        self.qr_sub = rospy.Subscriber('/uav' + str(num) + 
                '/mavros/qr', Bool, self.qr_cb)
        self.qr_flag = Bool()
        self.qr_flag.data = False

        #shutdown procedure
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        self.ctrl_c = True

    def qr_cb(self, msg):
        self.qr_flag = msg


    def publish(self):
        try:
            while not self.ctrl_c:
                # Read the length of the image as a 32-bit unsigned int. If the
                # length is zero, quit the loop
                image_len = struct.unpack('<L', 
                        self.connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break
                # Construct a stream to hold the image data and read the image
                # data from the connection
                image_stream = io.BytesIO()
                image_stream.write(self.connection.read(image_len))
                # Rewind the stream, open it as an image with PIL and do some
                # processing on it
                image_stream.seek(0)
                image = Image.open(image_stream)

                ### qr code dump ###
                # dump image
                if self.qr_flag.data:
                    cv2.imwrite("/home/stone3/images/image.jpg", image)
                    self.qr_flag.data = False

                #helmet detection
                np_im = numpy.array(image)
                np_im = cv2.cvtColor(np_im, cv2.COLOR_BGR2RGB)
                img_ROI, self.sensor_data.data[1], self.sensor_data.data[2], (h,w) = test(np_im)
                #print "x: ", centerX, "  y: ", centerY
                
                #write distance data to msg
                dist, addr = self.distance_socket.recvfrom(1024)
                self.sensor_data.data[3] = float(dist)

                #publish data
                self.drone_sensors_pub.publish(self.sensor_data)

        finally:
            self.connection.close()
            self.server_socket.close()
            self.distance_socket.close()





if __name__ == '__main__':
    rospy.init_node('sensor_pub_node')

    drone0_pub = raspi_pub(0)
    drone0_pub.publish()
