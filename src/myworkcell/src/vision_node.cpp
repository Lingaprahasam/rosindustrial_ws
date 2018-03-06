#include "ros/ros.h"
#include "fake_ar_publisher/ARMarker.h"
#include "myworkcell/LocalizePart.h"
// #include "tf/transform_listener.h"

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
	Localizer(ros::NodeHandle &nh)
	{
		// Topic
		ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
															&Localizer::visionCallback, this);

		// Service
		server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
	}

	// Topic
	void visionCallback(const fake_ar_publisher::ARMarkerConstPtr &msg)
	{
		last_msg_ = msg;
		// ROS_INFO_STREAM(last_msg_->pose.pose);
	}

	// Service
	bool localizePart(myworkcell::LocalizePart::Request &req,
					  myworkcell::LocalizePart::Response &res)
	{
		//   Read Last message from ar publisher
		fake_ar_publisher::ARMarkerConstPtr p = last_msg_;

		if (!p)
			return false;

		// // ==========================================================

		// // Transfer Frame

		// // ==========================================================

		// // to transform the over-the-wire format of geometry_msgs::Pose into a tf::Transform object
		// tf::Transform cam_to_target;
		// tf::poseMsgToTF(p->pose.pose, cam_to_target);
		
		// // This is looking for the transform from the requested object base_frame.
		// // each object from the work cell has base_frame
		// // For example 
		// // it could be from the work cell - 'world' is the base_frame of the work cell
		// // it could be any other object from the work cell ex: table - 'base_frame' of the table, the table could be placed in workcell with reference to 'world'
		// // to lookup the latest transform between the request.base_frame and the reference frame from the ARMarker message (which should be "camera_frame")
		// tf::StampedTransform req_to_cam;
		// listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);	
		
		// // try
		// // {
		// // 	listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);	
		// // }
		// // catch (tf::LookupException)
		// // {
		// // 	ROS_INFO("look up exception occured");
		// // }

		// // Here is the calculation of received pose from fake_ar_publisher to cameras base_frame to 
		// // finally with reference from the objects base_frame
		// // transform the object pose into the target frame.
		// tf::Transform req_to_target;
		// req_to_target = req_to_cam * cam_to_target;

		// // TF - This is used when transfer frame is used in the example.
		// tf::poseTFToMsg(req_to_target, res.pose);

		// Service - This is used when service is used in this example
		res.pose = p->pose.pose;		

		return true;
	}

	// Topic
	ros::Subscriber ar_sub_;
	// Service
	ros::ServiceServer server_;
	// Type - Pose
	fake_ar_publisher::ARMarkerConstPtr last_msg_;
	// TF listener
	// tf::TransformListener listener_;
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