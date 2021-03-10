#!/bin/sh
#xterm -e " roslaunch my_robot world.launch" &
#sleep 5
#xterm -e " roslaunch my_robot amcl.launch" &
#sleep 5
xterm -e " rosparam set goal1_x 3.0" 
xterm -e " rosparam set goal1_y 3.0" 
xterm -e " rosparam set goal2_x 2.0" 
xterm -e " rosparam set goal2_y 2.0" 
xterm -e " rosrun pick_objects pick_objects_test_node" 
#sleep 5
#xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
