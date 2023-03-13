#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <math.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Smb Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher cmd_publisher_;
	ros::Publisher vis_publisher_;

	float control_gain;

	void scanCallback(const sensor_msgs::LaserScan& msg);
};

} /* namespace */
