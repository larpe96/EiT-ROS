#pragma once

#include <iostream>
#include <vector>
#include <dynamic_reconfigure/server.h>
#include "pose_estimation/pose_est_srv.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include <string>
#include <ctime>
#include <fstream>
#include <ros/service_server.h>

#include <pose_estimation/detector.h>
#include <pose_estimation/PoseEstimationConfig.h>

class PoseEstimation
{
  public:
    typedef typename pose_estimation::PoseEstimationConfig
      DynamicReconfigureType;
    typedef typename dynamic_reconfigure::Server<DynamicReconfigureType>
      DynamicReconfigureServerType;

    PoseEstimation();
    void Initialize(const ros::NodeHandle &nh);
    void OnImage(const sensor_msgs::ImageConstPtr& img_rgb_msg, const sensor_msgs::ImageConstPtr& img_depth_msg);
    bool Estimate_pose(pose_estimation::pose_est_srv::Request   &req, pose_estimation::pose_est_srv::Response  &res);
    void getQuaternion(cv::Mat R, float Q[]);
    std::vector<cv::Mat> Detect(cv::Mat &img_rgb, cv::Mat &img_depth, cv::Mat &img_binary);
    void OnDynamicReconfigure(DynamicReconfigureType& config, uint32_t level);

  private:
    /// Node handler
    ros::NodeHandle nh_;
    /// ROS service
    ros::ServiceServer service_;
    /// Camera synchronizer policy
    using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    /// Camera synchronizer
    using CameraSynchronizer = message_filters::Synchronizer<CameraSyncPolicy>;
    std::shared_ptr<CameraSynchronizer> camera_sync_;
    /// Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> subscriber_rgb_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> subscriber_depth_;
    /// Dynamic reconfigure serer
    DynamicReconfigureServerType dynamic_reconfigure_server_;
    /// Stores current configuration
    pose_estimation::PoseEstimationConfig config_;
    /// Original RGB image
    cv::Mat img_rgb;
    /// Original depth image
    cv::Mat img_depth;
    /// Image with empty background used as reference image
    cv::Mat img_empty_background;

    cv::Mat img_diff_masked;
    cv::Mat img_diff;
    cv::Mat img_binary;

    float f_y = 574.0;
    float f_x = 574.0;
    cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4239851138500654, 0.9055161821251454, -0.01664533397227496, 0.464894891647513,
                                                    0.9036243510870013, 0.424190730960022, 0.05937386538029141, -0.4129218758926164,
                                                    0.06082479228194684, 0.01013250596365761, -0.9980970278318407, 0.5500317523427357,
                                                    0, 0, 0, 1);
    bool service_started = false;
};