#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>


ros::Publisher marker_pub;

int main( int argc, char** argv )
{
  // SETUP NODE AND MARKERS:
  // initialize node
  ros::init(argc, argv, "add_markers_test_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
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

  // showing marker at pick up location
  ROS_INFO("Showing marker at pick up location");
  // show marker at pick up location
  marker.pose.position.x = goal1_x;
  marker.pose.position.y = goal1_y;
  marker.pose.position.z = 0;
  marker_pub.publish(marker);
  ros::Duration(5.0).sleep();

  // delete marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ros::Duration(5.0).sleep();

  // showing marker at drop off location
  ROS_INFO("Showing marker at drop off location");
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;

  // show marker at pick up location
  marker.pose.position.x = goal2_x;
  marker.pose.position.y = goal2_y;
  marker.pose.position.z = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ros::Duration(5.0).sleep();

  // delete marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}
