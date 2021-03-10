#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects_test_node
  ros::init(argc, argv, "pick_objects_test_node");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  double goal1_x, goal1_y, goal1_a, goal2_x, goal2_y, goal2_a;
  ros::param::param<double>("goal1_x", goal1_x, 1.0);
  ros::param::param<double>("goal1_y", goal1_y, 1.0);
  ros::param::param<double>("goal2_x", goal2_x, 2.0);
  ros::param::param<double>("goal2_y", goal2_y, 2.0);

  // FIRST GOAL
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = goal1_x;
  goal.target_pose.pose.position.y = goal1_y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO_STREAM("Sending robot to x: " << goal1_x << " y: " << goal1_y << " phi: " << goal1_a);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached the first goal");
  else
    ROS_INFO("The robot failed to move to first goal");

  // SECOND GOAL
  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = goal2_x;
  goal.target_pose.pose.position.y = goal2_y;
  goal.target_pose.pose.orientation.w = 0.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO_STREAM("Sending robot to x: " << goal2_x << " y: " << goal2_y << " phi: " << goal2_a);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached the second goal");
  else
    ROS_INFO("The robot failed to move to second goal");

  return 0;
}
