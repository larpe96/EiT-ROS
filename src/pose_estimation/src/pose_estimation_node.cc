#include <pose_estimation.hpp>
#include "pose_estimation/pose_est_srv.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include <string>
#include <opencv/highgui.h>

PoseEstimation detector;
cv::Mat img;
cv::Mat img_depth;
cv::Mat background;

void getQuaternion(cv::Mat R, float Q[])
{
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);

    if (trace > 0.0)
    {
        float s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    }

    else
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        float s = sqrt(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}

void OnImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& img_depth_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
		cv_ptr_depth = cv_bridge::toCvCopy(img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	img = cv_ptr->image;
	img_depth = cv_ptr_depth->image;
	cv::imshow("rgb", img);
	cv::imshow("depth", img_depth);
  	cv::waitKey(1);
}

bool estimate_pose(pose_estimation::pose_est_srv::Request   &req,
		                pose_estimation::pose_est_srv::Response  &res)
{
	std::vector<cv::Mat> object_points;
	object_points = detector.Detect(img, img_depth);

	geometry_msgs::PoseArray posearray;
	posearray.header.stamp = ros::Time::now();

	for(int i = 0; i < object_points.size(); i++)
	{
		cv::Mat tmp_trans = object_points[i];
		float quat[4];
		getQuaternion(tmp_trans(cv::Rect(0,0,3,3)), quat);

		geometry_msgs::Pose p;
		p.position.x = tmp_trans.at<float>(0, 3);
		p.position.y = tmp_trans.at<float>(1, 3);
		p.position.z = tmp_trans.at<float>(2, 3);
    	p.orientation.x = quat[0];
		p.orientation.y = quat[1];
		p.orientation.z = quat[2];
		p.orientation.w = quat[3];

		posearray.poses.push_back(p);
		//std::cout << tmp_trans << std::endl;
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
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_rgb_sub, image_depth_sub);
  sync.registerCallback(boost::bind(&OnImage, _1, _2));

  // Detector
  cv::Mat img_background = cv::imread("/home/user/workspace/src/pose_estimation/src/backgroundEmpty.png"); //background.jpg");
  detector.calibrate_background(img_background);
//   background = cv::imread("background.png");
 cv::imshow("background", img_background);
  //detector.show_hist(detector.background_histogram);

  // Spin
  ros::ServiceServer service = nh.advertiseService("pose_est", estimate_pose);
  ROS_INFO("Ready to estimate relative position of the object.");
  ros::spin();
}
