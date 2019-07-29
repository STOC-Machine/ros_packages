#! /usr/bin/env python
import rospy
import csv 


import pathlib

DroneName = {'red': 0, 'blue': 1, 'purple': 2, 'orange': 3}
Action = {"fly": 0, "turn": 1, "stop": 2, "save": 3, "screenshot": 4, "special": 7}
MoveDirection = {"forward": 0, "backward": 1, "back": 1, "up": 2, "down": 3, 
"right": 4, "left": 5, "to": 6, "north":7, "south": 8, "east": 9, "west": 10}
# 3DTurn = {"x y": 0, "x z": 1, "y z": 2}
# Mode = {"attack": 0, "defense": 1}
TurnDirection = {"counter-clockwise": 0, "counter": 0, "clockwise": 1, "back": 2}

Port = 12459

def analyze_command(command):
    # Find the drone info
    words = command.split()
    words = [word.lower() for word in words]

    try:
        index = 0
        
        if words[index] in DroneName: 
            no_drone = int(DroneName[ words[index] ])
            index+=1
        else: # in case of redfly, etc.
            for name in DroneName:
                appearance = words[0].find(name)
                if appearance == 0:
                    no_drone = int(DroneName[ name ])
                    words[0] = words[0][len(name):]
                    # print("New", words[0])
        action = Action[ words[index]]

        if words[index] == "fly":
            direction = MoveDirection[ words[index+1]]
        elif words[index] == "turn":
            direction = TurnDirection[ words[index+1]]
        elif words[index] == "stop":
            direction = 0
        elif words[index] == "save":
            direction = 0
        elif words[index] == "screenshot":
            direction = 0
        elif words[index] == "special":
            direction = (1,1,1)

        return (no_drone, action, direction)
        # TODO: Consider velocity
    except KeyError:
        print("Invalid command")
        return (-1,0,0)
    except IndexError:
        print("Invalid command")
        return (-1,0,0)

import socket
# from cmd_pub import cmd_pub

def start_listen():
    # create cmd_pub object
    command_publisher = cmd_pub()
    # create socket connection
    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.bind(('0.0.0.0', Port))
    serv.listen(5)
    print "listening"
    while True:
        conn, addr = serv.accept()
        print "connected"
        from_client = ''
        while True:
            #print "d"
            data = conn.recv(4096)
            if not data: continue
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
                #print(command, "convert to: ", command_tmp)
                if command_tmp != (-1,0,0):
                    print("Valid command")
                    command_publisher.publish(command_tmp)
                    break
            #print "a"
            command_publisher.publish(analyze_command(message))
            #print "b"
            data = "" 
        #print "c"
        conn.close()

# Testing
# analyze_command("blue fly forward")
# analyze_command("red save me")
# analyze_command("orange stop")
# analyze_command("bluefly up")
# analyze_command("redfly to me")
# analyze_command("blue fly")

# analyze_command("blue FLY forward")
# analyze_command("red SAVE me")
# analyze_command("ORANGE stop")
# analyze_command("blue screenshot")
# analyze_command("red fly TO ME")
# analyze_command("blue fly")

########        main program        ########
if __name__ == "__main__":
    rospy.init_node('analyze_command_node')

    # run loop to listen for commands
    start_listen()

    test cmd_pub
    command_publisher = cmd_pub()
    command_publisher.publish(analyze_command("orange stop"))
    command_publisher.publish((0,1,1))






