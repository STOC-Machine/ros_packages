#! /usr/bin/env python
import rospy
import csv 
import time


### qr code stuff ###
from std_msgs.msg import Bool
qr_pub = rospy.Publisher('uav0/mavros/qr', Bool, queue_size=1)
qr_msg = Bool()
qr_msg = True


import pathlib

# DroneName = {'red': 0, 'blue': 1, 'purple': 2, 'orange': 3}
DroneAction = {"fly": 0, "turn": 1, "stop": 2, "save": 3, "screenshot": 4, "special": 7}
# Action = {"agree": 0}
MoveDirection = {"forward": 0, "backward": 1, "back": 1, "up": 2, "down": 3, 
"right": 4, "left": 5, "to": 6, "north":7, "south": 8, "east": 9, "west": 10}
# 3DTurn = {"x y": 0, "x z": 1, "y z": 2}
# Mode = {"attack": 0, "defense": 1}
Position = {"bottom right": 0, "bottom left": 1, "top right": 2, "top left": 3}
TurnDirection = {"counter-clockwise": 0, "counter": 0, "clockwise": 1, "forward": 2, "back": 3, "right": 4, "left": 5}

Port = 12459

def analyze_command(command):
    # Convert command into format:
    #      ()
    #          action (for drone or for computer to process image,
    #          direction_of_drone_movement)

    # Find the drone info
    words = command.split()
    words = [word.lower() for word in words]

    try:
        index = 0
        no_drone = -1
        action = 0
        direction = 0
        
        action = DroneAction[ words[index]]

        if words[index] == "fly":
            direction = MoveDirection[ words[index+1]]
        elif words[index] == "turn":
            direction = TurnDirection[ words[index+1]]
        elif words[index] == "stop":
            direction = 0
        elif words[index] == "save":
            direction = 0
        elif words[index] == "screenshot": ###
            direction = Position[words[index+1] + " " + words[index+2]]
        # elif words[index] == "special":
        #     direction = (1,1)

        return (action, direction)
        # TODO: Consider velocity
    except KeyError:
        print("Invalid command")
        return (-1, -1)
    except IndexError:
        print("Invalid command")
        return (-1, -1)

import socket
from cmd_pub import cmd_pub
from qr_detection import process_image

def start_listen():
    # create cmd_pub object
    command_publisher = cmd_pub()
    # create socket connection
    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.settimeout(None)
    serv.bind(('0.0.0.0', Port))
    serv.listen(5)
    print "listening"
    while True:
        conn, addr = serv.accept()
        print "connected"
        from_client = ''
        while True:
            #print "d"
            try:
                data = conn.recv(4096)
            except socket.timeout:
                break
            if not data: break
            # from_client += data
            #message = data.decode("utf-8")
            message = list(data[5:-1].split(", "))
            print("Receive message:")
            print(data)
            print(message )
            print(type(message))
            print(type(data))
            for command in message:
                command_tmp = analyze_command(command)
                if command_tmp[0] == 4: # Screenshot
                    print("Analyze screenshot")
                    qr_pub.publish(qr_msg)
                    time.sleep(3)
                    process_image("/home/stone3/images/image.jpg", command_tmp[1], "/home/stone3/images/image2.jpg", serv)
                    # print(command, "convert to: ", command_tmp)
                if command_tmp != (-1,-1):
                    print("Valid command")
                    command_publisher.publish(command_tmp)
                    break
            #print "a"
            #command_publisher.publish(analyze_command(message))
            #print "b"
            data = "" 
        print "disconnected"
        conn.close()

# Testing
# analyze_command("blue fly forward")
# analyze_command("save me")
# analyze_command("stop")
# analyze_command("fly up")
# analyze_command("fly to me")
# analyze_command("blue fly")

# analyze_command("blue FLY forward")
# analyze_command("SAve me")
# analyze_command("ORANGE turn")
# analyze_command("screenshot")
# analyze_command("red fly TO ME")
# analyze_command("blue fly")


########        main program        ########
if __name__ == "__main__":
    rospy.init_node('analyze_command_node')

    # run loop to listen for commands
    start_listen()

    #test qr code
    #qr_pub.publish(qr_msg)
    #time.sleep(3)
    #process_image("/home/stone3/images/image.jpg", 0, 
    #        "/home/stone3/images/image2.jpg", serv)
 







