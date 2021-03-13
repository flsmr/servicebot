# ROS Home Service Bot
Simulation of a robot within a Gazebo world which performs path finding in between two user-selected targets. This project is the final submission for the Udacity Robot Software Engineer Nanodegree. In a nutshell, this project joins some important building blocks within the world of robotics: Mapping of an unknown environment, Localization, as well as path planning within the ROS framework. Download this project to gain insights on how all these blocks interact and let the robot move around between user defined locations.
<img src="./img/home_service_bot_demo.gif" alt="Service robot finding target (Rviz and Gazebo)" width="800"/>

## Key Features
As a part of this project, a simple robot and a virtual environment of an apartment were designed within the standard physics simulation environment Gazebo. Within Gazebo, the robot moves around obeying the basic laws of physics and at the same time, corresponding sensor readings are simulated and reported to the ROS environment. Using the provided ROS packages, the following tasks can be accomplished:
* Creating a map while teleoperating the robot through the virtual environment (SLAM)
* Navigating the robot within the established map, by manually providing move-to locations using the Nav Goal within RVIZ
* Showing markers within RVIZ at user defined locations using the add_markers node
* Sending the robot to two subsequent locations using the pick_objects node
* Simulate a basic home service robot which first moves to a pick up location and further proceeds to a drop off location indicated by markers.

## Getting Started
The following steps require to have ROS (Robot Operating System) running (see [ROS Installation](https://www.ros.org/install/))
* Install required ros packages for loading maps, localization, navigation, and map creation
<pre>$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
$ sudo apt-get install ros-${ROS_DISTRO}-slam-gmapping</pre>
* install xterm (used within start up scripts)
<pre>$ sudo apt-get install xterm</pre>
* Create a new catkin workspace
<pre>$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace</pre>
* Download tele operation package into src folder
<pre>$ git clone https://github.com/ros-teleop/teleop_twist_keyboard</pre>
* Download and build this repo within src folder
<pre>$ git clone https://github.com/flsmr/servicebot.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash</pre>

## Running the scripts
As multiple packages need to be running at the same time, for convenience, several scripts were created which start all the nodes for you automatically. If you would like to load your own environment for the simulation or change the current environment ("apartment"), a new map needs to be created. For this purpose you can run the "test_slam.sh" script and after manually exploring the environment by steering the robot via keyboard, save the map via <pre># rosrun map_server map_saver -f <path/filename></pre>. The map creation process should like the image below.
![Creating a new map](./img/mapping.png?raw=true "Mapping")
* Map creation by controlling the robot via keyboard (map already exists for current environment - redo the mapping step if you changed the environment)
<pre>./src/servicebot/scripts/test_slam.sh</pre>
* Test navigation (use nav goal within rviz to set target pose and watch the adaptive monte carlo localization package do its work)
<pre>./src/servicebot/scripts/test_navigation.sh</pre>
* Test display of markers (subsequently displays two markers within the map in rviz - locations can be changed within script)
<pre>./src/servicebot/scripts/add_marker.sh</pre>
* Test robot move commands (sends the robot to two subsequent targets - locations can be changed within script)
<pre>./src/servicebot/scripts/pick_objects.sh</pre>
* Simulate home service robot (subsequently displays and sends robot to two targets - locations can be changed within script)
<pre>./src/servicebot/scripts/home_service.sh</pre>
![Service Bot Finding Path To Target](./img/service_bot.png?raw=true "Service Bot")
