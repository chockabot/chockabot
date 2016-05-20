turtlebot_teleop
================
ey ppl, to use chockabot:

on robot:
roslaunch turtlebot_bringup minimal.launch 
roslaunch turtlebot_navigation 3d_amcl.launch map_file:=/tmp/my_map.yaml 
    (if you have to create a map use this: http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)
    ... and this (http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map#Localize_the_TurtleBot)
if you have the arduino plugged to the RIGHT side usb on the laptop, run the following to set it up:
   sudo chmod 777 /dev/ttyAMC0
   (click the little reset button on the arduino)
   rosrun rosserial_python serial_node.py /dev/ttyACM0

on computer:
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen (if you want to localize the robot)
roslaunch turtlebot_teleop alvar.launch (for ar marker tracking)
roslaunch turtlebot_teleop turtlebot_feducial_2.launch (for the service to get/set a bin)