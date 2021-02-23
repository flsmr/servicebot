#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include "pick_objects/GoToPosition.h"


ros::ServiceClient goToPositionClient;
ros::Publisher marker_pub;

void showMarker(float xpos, float ypos) {
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
    marker.pose.position.x = xpos;
    marker.pose.position.y = ypos;
    marker.pose.position.z = 0;
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

    marker.lifetime = ros::Duration(5.0);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
/*
      if (!ros::ok())
      {
        return 0;
      }
*/
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // show marker at location
    marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  // set pick up and drop off locations
  float pickupXpos = -1.0;
  float pickupYpos = -1.0;
  float pickupZrot = 1.0;

  float dropoffXpos = -2.0;
  float dropoffYpos = -2.0;
  float dropoffZrot = 2.0;

  ros::init(argc, argv, "add_markers_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // set up client for requesting position change of robot
  goToPositionClient = n.serviceClient<pick_objects::GoToPosition>("/pick_objects/go_to_position");

  pick_objects::GoToPosition goToPositionService;

  // send robot to first location
  showMarker(pickupXpos,pickupYpos);
  goToPositionService.request.xpos = pickupXpos;
  goToPositionService.request.ypos = pickupYpos;
  goToPositionService.request.zrot = pickupZrot;
  if (! goToPositionClient.call(goToPositionService))
    ROS_ERROR("Failed to call service go_to_location");

  // send robot to second location
  showMarker(pickupXpos,pickupYpos);
  goToPositionService.request.xpos = dropoffXpos;
  goToPositionService.request.ypos = dropoffYpos;
  goToPositionService.request.zrot = dropoffZrot;
  if (! goToPositionClient.call(goToPositionService))
    ROS_ERROR("Failed to call service go_to_location");


/*
    // sleep for 5 seconds
    ros::Duration(5.0).sleep();

    // delete marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // sleep for 5 seconds
    ros::Duration(5.0).sleep();

    // show marker at drop off zone
    shape = visualization_msgs::Marker::ARROW;
    marker.pose.position.x = 2;
    marker.pose.position.y = 2;
    marker.pose.position.z = 0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
*/
    r.sleep();
}
