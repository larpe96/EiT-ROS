# EiT

## Initialize Environment 
```bash
cd Eit-ROS
source devel/setup.bash 
catkin_make
```

## Launch system 
```bash
cd folder/containing/launch-file 
roslaunch <package_name> <launch_file>
```

## Installation of UR_driver

```bash

# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
source devel/setup.bash
```
Test the Ur driver:
This guide assumes the following setup
| host_ip |  Robot_ip  |
|:------------:|:--------:|
| 192.168.1.61   | 192.168.1.68 |

```bash
# Start the ur5e_node
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.68
```
Run the urcap on the robot (local mode)


In another terminal for visual:
```bash
roslaunch ur_robot_driver example_rviz.launch 
```
In a third terminal try:
```bash
rosrun ur_robot_driver test_move 
```
