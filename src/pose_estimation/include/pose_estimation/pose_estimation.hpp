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
#include <opencv/highgui.h>
#include <ctime>
#include <fstream>
#include <ros/service_server.h>

#include <pose_estimation/detector.h>
#include <pose_estimation/PoseEstimationConfig.h>

class PoseEstimation
{
    public:
        PoseEstimation();
        float findMedian(std::vector<float> a, int n);
        std::vector<float> depth_within_perimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img);
        std::vector<cv::Mat> Detect(cv::Mat &img, cv::Mat &depth_img);
        void calibrate_background(cv::Mat &background_img);
        void backprojectHistogram(cv::Mat &img);
        void detect_circles(cv::Mat &img);
        cv::Mat apply_mask(cv::Mat img);
        void show_hist(cv::MatND hist);
        std::vector<cv::Mat> convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, float img_w, float img_h);
        void drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles, cv::Scalar color, int radius);

    PoseEstimation();
    void Initialize(const ros::NodeHandle &nh);
    void OnImage(const sensor_msgs::ImageConstPtr& img_rgb_msg, const sensor_msgs::ImageConstPtr& img_depth_msg);
    bool Estimate_pose(pose_estimation::pose_est_srv::Request   &req, pose_estimation::pose_est_srv::Response  &res);
    void getQuaternion(cv::Mat R, float Q[]);
    std::vector<cv::Mat> Detect(cv::Mat &img_rgb, cv::Mat &img_depth, cv::Mat &img_binary);
    void OnDynamicReconfigure(DynamicReconfigureType& config, uint32_t level);

    private:
        cv::Mat img;
        cv::Mat backProj;
        cv::Mat bin_image;
        cv::Mat background;

        std::vector<cv::Point3f> center_points;
        float THRESH_BACKPROJ2BIN = 10; //10
        int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
        int num_hist_bin = 180;            //Number of bin in the histogram
        float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
        const float* channel_ranges[1] = {h_range};
        std::vector<cv::Vec3f> debug_circles; //Stores the complete information about the circles detected in the last image.
        cv::Mat img_masked;
        cv::Rect mask_rect = cv::Rect(1, 1, 620, 470);
        float f_y = 574.0;
        float f_x = 574.0;
        cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4239851138500654, 0.9055161821251454, -0.01664533397227496, 0.464894891647513,
 0.9036243510870013, 0.424190730960022, 0.05937386538029141, -0.4129218758926164,
 0.06082479228194684, 0.01013250596365761, -0.9980970278318407, 0.5500317523427357,
 0, 0, 0, 1);

        cv::Mat base2camera = (cv::Mat_<float>(4,4) << -0.3200615474180835, 0.906443881953242, 0.2755360134971088, 0.5131311099627459,
                                                        0.9247229143918688, 0.3621450629584728, -0.1172112834731024, -0.2413966376231433,
                                                        -0.2060294577553179, 0.2172796406577029, -0.9541181374927892, 0.5905108421276393,
                                                        0, 0, 0, 1);
};
