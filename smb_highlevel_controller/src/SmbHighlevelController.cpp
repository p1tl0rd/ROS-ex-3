#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  std::string topic;
  int queue_size;
  if ( !nodeHandle.getParam("subscriber_topic", topic) 
      || !nodeHandle.getParam("queue_size", queue_size) ) 
  {
      ROS_ERROR("Could not find subscriber params!"); 
      ros::requestShutdown();
  }

  control_gain = 0.5;

  if (!nodeHandle_.getParam("control_gain", control_gain)) {
    ROS_ERROR("No control_gain parameter defined!");
  }
  // create subscriber
  subscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::scanCallback, this);

  // createt publisher
  cmd_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  vis_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  
  ROS_INFO("Smb highlevel controller node launched!");
  // sleeping for a few seconds
  ros::Duration(3).sleep();
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
  int size = msg.ranges.size();
  float min = msg.range_max;
  int minIndex = 0;

  // tim khoang cach nho nhat tu vector msg.ranges
  for (int i = 0; i < size; i++) {
    if (msg.ranges.at(i) < min && msg.ranges.at(i) > msg.range_min) {
      min = msg.ranges.at(i);
      minIndex = i;
    }
  }

  float angle = msg.angle_min + (msg.angle_increment * minIndex);
  float x = min * cos(angle);
  float y = min * tan(angle);
  

  geometry_msgs::Twist cmd;

  // DK robot den pillar
  if (min > 1){
    cmd.linear.x = 1; //toc do
    cmd.angular.z = control_gain * atan2(y,x); //angle
  }
  else{
    cmd.linear.x = 0;
    cmd.angular.z = 0;
  }

  cmd_publisher_.publish(cmd);

  // visualize pillar in Rviz
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();

  marker.ns = "pillar";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (min + 1) * cos(angle);
  marker.pose.position.y = -(min) * sin(angle);
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0.0;

  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  vis_publisher_.publish(marker);

  ROS_INFO_STREAM("Minimum min: " << min);
}
} /* namespace */
