#include <pose_estimation/pose_estimation.hpp>
#include "pose_estimation/pose_est_srv.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include <string>
#include <opencv/highgui.h>

PoseEstimation detector;
cv::Mat img;

void OnImage(const sensor_msgs::ImageConstPtr& img_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	img = cv_ptr->image;
	cv::imshow("test", img);
	cv::waitKey(1);
}

bool estimate_pose(pose_estimation::pose_est_srv::Request   &req,
		                pose_estimation::pose_est_srv::Response  &res)
{
	std::vector<cv::Point2f> object_points;
	object_points = detector.Detect(img);

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
  ros::Subscriber subscriber = nh.subscribe("/camera/rgb/image_raw", 1, OnImage);

  // Detector
  cv::Mat img_background = cv::imread("../EiT-ROS/src/pose_estimation/src/background.jpg");
  detector.calibrate_background(img_background);
  //detector.show_hist(detector.background_histogram);

  // Spin
  ros::ServiceServer service = nh.advertiseService("pose_est", estimate_pose);
  ROS_INFO("Ready to estimate relative position of the object.");
  ros::spin();
}
