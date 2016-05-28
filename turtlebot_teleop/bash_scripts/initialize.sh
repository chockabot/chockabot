#!/bin/bash
# runs stuff from turtlebot
setrobot loggerhead & ssh turtlebot@loggerhead 'bash -s' < initialize_turtlebot.sh &
sleep(15)
setrobot loggerhead
echo "running interactive markers.."
rosrun turtlebot_teleop turtlebot_marker &
sleep(2)
echo "Running RVIZ"
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen &
sleep(4)
echo "Running alvar.launch"
roslaunch turtlebot_teleop alvar.launch &
sleep(3)
echo "Running the feducial server"
roslaunch turtlebot_teleop turtlebot_feducial_2.launch &
sleep(3)
echo "Running the app"
roslaunch turtlebot_teleop app.launch &
sleep(3)
echo "Running the brain.."
roslaunch turtlebot_teleop chockabot_brain.launch
