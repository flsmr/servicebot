#!/bin/sh
# launch robot, gazebo world and start rviz for visualization
xterm -e " roslaunch my_robot world.launch" &
sleep 5
# start mapping
xterm -e " rosrun gmapping slam_gmapping" &
sleep 5
# start keyboard control for robot
xterm -e " roslaunch my_robot teleop.launch"
# save map file
# rosrun map_server map_saver -f <path/filename>
