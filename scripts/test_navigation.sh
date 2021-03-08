#!/bin/sh
# launch robot in gazebo world and start rviz
xterm -e " roslaunch my_robot world.launch" &
sleep 5
# launch localization and navigation
xterm -e " roslaunch my_robot amcl.launch" &
