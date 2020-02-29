# ros_packages
ROS packages to control drone flight

These are the main ROS packages that will be used to control the drone
in mission 8. These packages must be in the /src directory of your catkin
work space. Directions on how to use below.

## Directions on Use

Here are instructions on how to setup a development enviroment and use this code to control a simulated drone. We will go over how to:

* Install ROS and other programs
* Setup catkin workspace
* Setup PX4 firmware
* Simulating the drone
* Code Explaination

### Install ROS and Other Programs

You must be working on a Linux operating system to install these programs and simulate a drone. If you do not have a machine with a linux operating system, a virtual machine would be your best choice then. The key components that you need to install are:

* ROS ([install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu))
* Mavros ([install instructions](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html))
  * also install GeographicLib (same instructions)
  
If you want to make the px4 firmware code, you will also need to install other dependencies. To install these programs and set up your machine to compile the px4 flight stack, follow the intructions on the [px4 development guide](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html) under the ROS/Gazebo heading. You will download a shell script which will install everything you need. If you run into problems, everything can be intalled manually. This takes more time and effort, however.
  
### Setup Catkin Workspace

The catkin workspace is the directory that you will write and compile your code in. First, make the directory
```
mkdir -p catkin_ws/src
```
Next, you need to clone this repository into the `src/` folder.
```
cd ~/catkin_ws/src
git clone https://github.com/STOC-Machine/ros_packages.git
```
In the base of your workspace, run `catkin_make` in order to initialize it as you workspace. This also compiles all the packages you just downloaded using `git clone`.
```
cd ~/catkin_ws
catkin_make
```
Finally, you need to add this workspace to your enviroment, so ROS knows that these new packages exsist. You will add this command to your `.bashrc` so its added to your shell enviroment everytime you open a new terminal.
```
cd ~
echo 'source ~/catkin_ws/devel/setup.bash' >> .bashrc
```

### Setup PX4 Firmware

The px4 firmware code allows us to simulate a drone on our computer by running its software on our machine rather than a physical drone. You can try to clone the [official px4 repo](https://github.com/PX4/Firmware) yourself onto your machine somewhere. You may run into many issues, however, as there are many dependencies, it must be made, and new releases tend to mess things up.

To easily get started, I reccomend cloning our [px4-firmware](https://github.com/STOC-Machine/px4-firmware) repo. This is an older version of the code, but is already compiled and tested, so it should work. All you have to do is clode it to your machine.
```
cd ~
git clone https://github.com/STOC-Machine/px4-firmware.git
```
Before running any simulation, you must wrap px4 as a ROS package. This must be done everytime you close out of your terminal and open a new terminal.
```
cd ~/px4-firmware
source launch-common.sh
```

### Simulating the Drone
These are the directions on how to simulate a drone running on your computer in gazebo. You will be able to control the drone by running a certain python script.

Start by opening up a terminal session, and wrapping the px4-firmware code.
```
source px4-firmware/launch-common.sh
```
Next, you will launch a roslaunch file that will start gazebo, the sitl drone, and mavros.
```
roslaunch offb_py simulation_main.launch
```
This will open the program gazebo, and you should see a drone sitting in the middle of the world. To fly the drone, we must run a node that controls it. Open up a new terminal and run the program `multi_drone.py`.
```
rosrun offb_py multi_drone.py
```
The program will prompt you if you want to set the mode to offboard. Enter any number and hit enter. It will then ask you if you want to arm the drone. Again, enter a number and hit enter. The drone should takeoff and hover in the air.

To control the drone's movements, we need to run another program that sends messages about how the drone should move. While keeping your current terminals running, open up a new terminal and run the `command_pub.py` code.
```
rosrun offb_py command_pub.py
```
You will recieve prompts about how you want the drone to move. Enter numbers corresponding to the prompts and hit enter. You will recieve two prompts for each command.

When you are done, you can Ctrl-C the programs to quit, and exit out of gazebo. Some of the terminals may freeze or give alot of errors. Just exit out of the terminal, and open a new terminal to continue.

### Brief Code Explanantion
The main files that we are working with here are:
* launch-common.sh
  * 
* simulation_main.py
* multi_drone.py
* command_pub.py

Glance through each file to try and get a feel for how things work together. Feel free to change the code or add new files. Just don't push any of your changes back to master. Creating your own branch for messing around with things could be useful.

## Questions and Comments
I know that these instructions may be very confusing at many points and may have numerous holes of information. I intend this to be a rough draft, and will be continually making this documentation more consise and thorough. Please, leave comments all over this page, where you think more information, or better instructions are needed!

For any questions you have, please reach out to me, and I can work with you in conquering the beasts of ROS and px4. You can email me (stone3@stolaf.edu), or pull me aside at any meeting.

Happy Droneing!!
