#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pick_objects/GoToPosition.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool moveToPosition(pick_objects::GoToPosition::Request& targetPose, pick_objects::GoToPosition::Response& responseMsg) {
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 1 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // SEND GOAL
  // ^^^^^^^^^^^^^^^
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = targetPose.xpos;
  goal.target_pose.pose.position.y = targetPose.ypos;
  goal.target_pose.pose.orientation.w = targetPose.zrot;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending robot to location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached location");
    //responseMsg = "Robot reached location";
    return true;
  }
  else {
    ROS_INFO("Robot could not reach location!");
    //responseMsg = "Robot could not reach location!";
    return false;
  }
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  // register service go_to_position
  ros::ServiceServer goToPositionService = n.advertiseService("/pick_objects/go_to_position",moveToPosition);

  ros::spin();
  return 0;
}
