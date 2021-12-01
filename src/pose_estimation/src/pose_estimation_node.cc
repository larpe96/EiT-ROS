#include <pose_estimation/pose_estimation.hpp>


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pose_estimation");
  auto nh = ros::NodeHandle("~");

  // Detector
  PoseEstimation detector;
  detector.Initialize(nh);


  // Spin
  ros::spin();
}
