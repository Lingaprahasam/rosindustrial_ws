#include "ros/ros.h"
#include "fake_ar_publisher/ARMarker.h"

// /**
//  * This tutorial demonstrates simple sending of messages over the ROS system.
//  */
// int main(int argc, char *argv[])
// {
// 	/**
// 	 * The ros::init() function needs to see argc and argv so that it can perform
// 	 * any ROS arguments and name remapping that were provided at the command line.
// 	 * For programmatic remappings you can use a different version of init() which takes
// 	 * remappings directly, but for most command-line programs, passing argc and argv is
// 	 * the easiest way to do it.  The third argument to init() is the name of the node.
// 	 *
// 	 * You must call one of the versions of ros::init() before using any other
// 	 * part of the ROS system.
// 	 */
// 	ros::init(argc, argv, "vision_node");

// 	/**
// 	 * NodeHandle is the main access point to communications with the ROS system.
// 	 * The first NodeHandle constructed will fully initialize this node, and the last
// 	 * NodeHandle destructed will close down the node.
// 	 */
// 	ros::NodeHandle n;
	
// 	ROS_INFO("Hello, World!");
	
// 	ros::spin();

// }

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
      &Localizer::visionCallback, this);
  }

  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
  {
      last_msg_ = msg;
      ROS_INFO_STREAM(last_msg_->pose.pose);
  }

  ros::Subscriber ar_sub_;
  fake_ar_publisher::ARMarkerConstPtr last_msg_;
};

int main(int argc, char *argv[])
{
	// * You must call one of the versions of ros::init() before using any other
	//  * part of the ROS system.
	ros::init(argc, argv, "vision_node");

	// 	/**
	// 	 * NodeHandle is the main access point to communications with the ROS system.
	// 	 * The first NodeHandle constructed will fully initialize this node, and the last
	// 	 * NodeHandle destructed will close down the node.
	// 	 */
	ros::NodeHandle n;


	// The Localizer class provides this node's ROS interfaces
  	Localizer localizer(n);

  	ROS_INFO("Vision node starting");

	ros::spin();

}