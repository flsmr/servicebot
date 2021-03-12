# ROS Home Service Bot
Simulation of a robot within a Gazebo world which performs path finding in between two user-selected targets. This project is the final submission for the Udacity Robot Software Engineer Nanodegree. In a nutshell, this project joins some important building blocks within the world of robotics: Mapping of an unknown environment, Localization, as well as path planning within the ROS framework. Download this project to gain insights on how all these blocks interact and let the robot move around between user defined locations.
![Service Bot Finding Path To Target](./img/service_bot.png?raw=true "Service Bot")

## Key Features
As part of this project, a simple robot and a virtual environment of an apartment were designed within the standard physics simulation environment Gazebo. Within Gazebo, the robot moves around obeying the basic laws of physics and at the same time, corresponding sensor readings are simulated and reported to the ROS environment. Using the provided ROS packages, the following tasks can be accomplished:
* Creating a map while teleoperating the robot through the virtual environment (SLAM)
* Navigating the robot within the established map, by manually providing move-to locations using the Nav Goal within RVIZ
* Showing markers within RVIZ at user defined locations using the add_markers node
* Sending the robot to two subsequent locations using the pick_objects node
* Simulate a basic home service robot which first moves to a pick up location and further proceeds to a drop off location indicated by markers.

## Getting Started
This following step require to have ROS (Robot Operating System) running (see [ROS Installation](https://www.ros.org/install/))
* Create a new workspace a new workspace
<pre>$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo apt-get update
$ cd ~/catkin_ws/src</pre>
* Installing ROS packages
<pre>$ git clone https://github.com/ros-perception/slam_gmapping</pre>
* Installing packages from this repo

* Installing dependencies and building the project
<pre>$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosdep -i install gmapping
$ catkin_make
$ source devel/setup.bash</pre>
