#!/bin/bash
echo "Running Bringup..."
roslaunch turtlebot_bringup minimal.launch &
sleep(3)
echo "Running the 3d_amcl"
roslaunch turtlebot_navigation 3d_amcl.launch map_file:=/home/turtlebot/chockabot_workspace/src/turtlebot_teleop/map/my_map.yaml.yaml &
sleep(8)
echo "Setting up the arduino"
echo motion6 | sudo -S chmod 777 /dev/ttyACM0 &
sleep(1)
rosrun rosserial_python serial_node.py /dev/ACM0 &
sleep(6)
echo
echo
echo "Okay, good to go...."
echo
echo