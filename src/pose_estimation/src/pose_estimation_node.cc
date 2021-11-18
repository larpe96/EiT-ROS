#include <pose_estimation/pose_estimation.hpp>


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pose_estimation");
  //ros::NodeHandle nh;
  auto nh = ros::NodeHandle("~");

  // Detector
  PoseEstimation detector;
  detector.Initialize(nh);
/*
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_rgb_sub, image_depth_sub);
  sync.registerCallback(boost::bind(&OnImage, _1, _2));


  // Detector
  cv::Mat img_background = cv::imread("/home/mdn/bor/EiT-ROS/src/pose_estimation/src/backgroundBrown.png"); //background.jpg");
  detector.calibrate_background(img_background);
  background = cv::imread("background.png");
  cv::imshow("background", background);
  //detector.show_hist(detector.background_histogram);

  // Spin
  ros::ServiceServer service = nh.advertiseService("pose_est", estimate_pose);
  ROS_INFO("Ready to estimate relative position of the object.");
  */

  // Spin
  ros::spin();
}
