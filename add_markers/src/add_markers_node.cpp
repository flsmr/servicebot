#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include "pick_objects/GoToPosition.h"


ros::ServiceClient goToPositionClient;
ros::Publisher marker_pub;

int main( int argc, char** argv )
{
  // SETUP NODE AND MARKERS:
  // initialize node
  ros::init(argc, argv, "add_markers_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok()) return 0;
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // PARAMETERS:
  // set pick up and drop off locations
  double goal1_x, goal1_y, goal1_a, goal2_x, goal2_y, goal2_a;
  ros::param::param<double>("goal1_x", goal1_x, 0.4);
  ros::param::param<double>("goal1_y", goal1_y, -6.25);
  ros::param::param<double>("goal1_a", goal1_a, 0.0);
  ros::param::param<double>("goal2_x", goal2_x, -2.85);
  ros::param::param<double>("goal2_y", goal2_y, -0.3);
  ros::param::param<double>("goal2_a", goal2_a, 1.57);

  // set up client for requesting position change of robot
  goToPositionClient = n.serviceClient<pick_objects::GoToPosition>("/pick_objects/go_to_position");

  pick_objects::GoToPosition goToPositionService;

  // Wait 1 sec for move_base action server to come up
  while(!ros::service::waitForService("/pick_objects/go_to_position",ros::Duration(3.0))){
    if (!ros::ok()) return 0;
    ROS_INFO("Waiting for the goToPositionService to come up");
  }

  // prepare markers
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // ROBOT -> PICK UP
  ROS_INFO("Sending robot to pick up location");
  // show marker at pick up location
  marker.pose.position.x = goal1_x;
  marker.pose.position.y = goal1_y;
  marker.pose.position.z = 0.0;
  marker_pub.publish(marker);

  // send robot to pickup location
  goToPositionService.request.xpos = goal1_x;
  goToPositionService.request.ypos = goal1_y;
  goToPositionService.request.zrot = goal1_a;
  if (! goToPositionClient.call(goToPositionService)) {
    ROS_ERROR("Failed to call service go_to_location");
  } else {
    ROS_INFO("Robot reached pick up location -> picking up...");
    // delete marker when robot reaches pickup zone and sleep 5 seconds
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    ROS_INFO("Finished picking up");

    // ROBOT -> DROP OFF
    ROS_INFO("Sending robot to drop off location");
    // send robot to drop off location
    goToPositionService.request.xpos = goal2_x;
    goToPositionService.request.ypos = goal2_y;
    goToPositionService.request.zrot = goal2_a;
    if (! goToPositionClient.call(goToPositionService)) {
      ROS_ERROR("Failed to call service go_to_location");
    } else {
      // show marker at drop off location
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = goal2_x;
      marker.pose.position.y = goal2_y;
      marker.pose.position.z = 0;
      marker_pub.publish(marker);
      ROS_INFO("Robot reached drop off location -> task completed");
    }
  }
  r.sleep();
}
