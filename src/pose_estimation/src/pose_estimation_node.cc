#include <pose_estimation/pose_estimation.hpp>
#include "pose_estimation/pose_est_srv.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include <string>

PoseEstimation detector;

bool estimate_pose(pose_estimation::pose_est_srv::Request   &req,
		                pose_estimation::pose_est_srv::Response  &res)
{
	sensor_msgs::ImageConstPtr img_msg =  ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw", ros::Duration(10));

	// Convert msg to CvImage
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	std::vector<cv::Point2f> object_points;
	object_points = detector.Detect(cv_ptr->image);

	geometry_msgs::PoseArray posearray;
	posearray.header.stamp = ros::Time::now();

	for(int i = 0; i < object_points.size(); i++)
	{
		geometry_msgs::Pose p;
		p.position.x = object_points[i].x;
    p.orientation.x = object_points[i].y;
		posearray.poses.push_back(p);
	}
	res.rel_object_poses = posearray;

	std::string out_str = "Number of detected objects: " + std::to_string(object_points.size());
	ROS_INFO_STREAM(out_str);
	return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  // Detector
  cv::Mat img_background = cv::imread("../EiT_workspace/src/pose_estimation/src/background.jpg");
  detector.calibrate_background(img_background);

  // Spin
  ros::ServiceServer service = nh.advertiseService("pose_est", estimate_pose);
  ROS_INFO("Ready to estimate relative position of the object.");
  ros::spin();
}