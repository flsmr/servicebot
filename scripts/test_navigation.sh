#!/bin/sh
# launch robot, gazebo world and start rviz for visualization
xterm -e " roslaunch my_robot world.launch" &
sleep 5
# launch localization and navigation
xterm -e " roslaunch my_robot amcl.launch" &
