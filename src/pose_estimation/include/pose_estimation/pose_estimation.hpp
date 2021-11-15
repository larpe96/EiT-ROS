#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
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
#include <opencv/highgui.h>
#include <ctime>
#include <fstream>
#include <ros/service_server.h>


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
    void calibrate_background(std::string background_path);

    bool Estimate_pose(pose_estimation::pose_est_srv::Request   &req, pose_estimation::pose_est_srv::Response  &res);
    void getQuaternion(cv::Mat R, float Q[]);
    std::vector<cv::Mat> Detect(cv::Mat &img_rgb, cv::Mat &img_depth);
    cv::Mat apply_mask(cv::Mat img);
    std::vector<float> depth_within_perimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img);
    float findMedian(std::vector<float> a, int n);
    std::vector<cv::RotatedRect> find_rotated_rects(std::vector<std::vector<cv::Point>> contours);
    std::vector<cv::Mat> convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, int img_w, int img_h);
    void OnDynamicReconfigure(DynamicReconfigureType& config, uint32_t level);

    void show_hist(cv::MatND hist);
    void drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles, cv::Scalar color, int radius);

  private:
    /// Node handler
    ros::NodeHandle nh_;
    /// ROS service
    ros::ServiceServer service_;
    /// ROS subscriber sync policy
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    using CameraSynchronizer = message_filters::Synchronizer<CameraSyncPolicy>;
    std::shared_ptr<CameraSynchronizer> camera_sync_;
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
    // Calibrate background
    cv::Mat img_background;
    cv::Mat background_img_HSV;
    cv::MatND background_histogram;
    int num_hist_bin = 180;            //Number of bin in the histogram
    int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
    float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
    const float* channel_ranges[1] = {h_range};
    // Estimate pose
    std::vector<cv::Mat> object_points;
    geometry_msgs::PoseArray posearray;
    std::string out_str;
    // Detect
    time_t current_time;
    std::string id;
    std::string id_path;
    cv::Mat img_diff_masked;
    cv::Mat img_diff;
    cv::Mat img_diff_masked_grey;
    cv::Mat img_binary;
    cv::Mat img_binary_erode;
    cv::Mat img_binary_dilate;
    //cv::Mat structuring_element;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<float> depth;
    std::vector<cv::RotatedRect> rot_rects;
    cv::RNG rng;
    std::vector<cv::Mat> object_trans;
    // Apply mask
    cv::Mat masked;
    cv::Rect mask_rect;
    // Depth within parameter
    cv::Mat filledImage;
    // Convert to transforms
    float f_y = 574.0;
    float f_x = 574.0;
    cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4092872843859879, 0.9119913122596123, -0.02749118409475548, 0.4613890083554278,
                                                    0.9118304765329119, 0.4099132124339962, 0.02315902281622022, -0.396963379231898,
                                                    0.03238982719471067, -0.01558860593609109, -0.9993537384026068, 0.5628008106183637,
                                                    0, 0, 0, 1);
    bool service_started = false;

/*
    cv::Mat backProj;
    std::vector<cv::Point3f> center_points;
    float THRESH_BACKPROJ2BIN = 10;
    int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
    int num_hist_bin = 180;            //Number of bin in the histogram
    float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
    const float* channel_ranges[1] = {h_range};
    std::vector<cv::Vec3f> debug_circles; //Stores the complete information about the circles detected in the last image.
*/

};