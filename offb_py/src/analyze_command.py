import csv 

# csv file name 

# 0: outcome, 5: batter, 7: inning, 10: top, which team is playing
import pathlib

DroneName = {'red': 0, 'blue': 1, 'purple': 2, 'orange': 3} 
Action = {"fly": 0, "turn": 1, "stop": 2, "save": 3, "special": 4} # balance, screenshot, position of images, turn front
MoveDirection = {"forward": 0, "backward": 1, "up": 2, "down": 3, "right": 4, "left": 5, "to": 6}
# 3DTurn = {"x y": 0, "x z": 1, "y z": 2}
TurnDirection = {"counter": 0, "clockwise": 1}

Port = 12459

def analyze_command(command):
    # Find the drone info
    words = command.split()
    
    # TODO: Catch error
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
        elif words[index] == "special":
            direction = (1,1,1)
        # Obtain commands

        # Obtain direction
        # start = words.find("fly")
        # direction = Direction[ words[start+1]]

        print((no_drone, action, direction))
        return (no_drone, action, direction)
        # TODO: Consider velocity
    except KeyError:
        print("Invalid command")
        return (0,0,0)
    except IndexError:
        print("Invalid command")
        return (0,0,0)

import socket

def start_listen():
    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.bind(('0.0.0.0', Port))
    serv.listen(5)
    while True:
        conn, addr = serv.accept()
        from_client = ''
        while True:
            data = conn.recv(4096)
            if not data: continue
            # from_client += data
            message = data.decode("utf-8")[4:]
            print("Receive message:")
            print(data.decode("utf-8")[4:] )
            print("Convert to: ", analyze_command(message))
        conn.close()

# start_listen()

# def send_message():
#     ## Connect to an IP with Port, could be a URL
#     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock.connect(('192.168.0.165', 12459))
#     ## Send some data, this method can be called multiple times
#     sock.send(bytes("Twenty-five bytes to send", 'utf-8'))
#     ## Receive up to 4096 bytes from a peer
#     sock.recv(4096)
#     ## Close the socket connection, no more data transmission
#     # sock.close()

# send_message()
analyze_command("blue fly forward")
analyze_command("red save me")
analyze_command("orange stop")
analyze_command("bluefly up")
analyze_command("redfly to me")
analyze_command("blue fly")






