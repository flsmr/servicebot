#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/corridor.world
#xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm -e " roslaunch my_robot world.launch" &
sleep 5
#xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
xterm -e " roslaunch my_robot amcl.launch" &
sleep 5
#xterm -e " roslaunch my_robot localization.launch" &
#xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
#xterm -e " roslaunch my_robot localization.launch" &
sleep 5
#xterm -e " rosrun pick_objects pick_objects_node" &
sleep 5
#xterm -e " rosrun add_markers add_markers_node"
