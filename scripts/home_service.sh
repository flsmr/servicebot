#!/bin/sh
# launch robot, gazebo world and start rviz for visualization
xterm -e " roslaunch my_robot world.launch" &
sleep 5
# launch localization and navigation
xterm -e " roslaunch my_robot amcl.launch" &
sleep 5
# enter individual goals if desired (angle in rad)
#xterm -e " rosparam set goal1_x 0.4" 
#xterm -e " rosparam set goal1_y -6.25" 
#xterm -e " rosparam set goal1_a 0.0" 
#xterm -e " rosparam set goal2_x -2.85" 
#xterm -e " rosparam set goal2_y -0.3" 
#xterm -e " rosparam set goal2_a 1.57" 
# mark two subsequent goals and send out robot
xterm -e " rosrun pick_objects pick_objects_node" &
sleep 5
xterm -e " rosrun add_markers add_markers_node" 

