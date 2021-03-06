#pragma once



// Custom Services 
#include "pose_estimation/pose_est_srv.h"
#include "classifier_pkg/classify_detections.h"
#include "classifier_pkg/Classification_params.h"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include <ros/service_server.h>
#include <dynamic_reconfigure/server.h>

// STD includes
#include <string>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

// Headers
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
    bool callClassifier(std::vector<cv::RotatedRect> rot_rects, std::vector<std::string> &labels, std::vector<int> &mask);
    void getQuaternion(cv::Mat R, float Q[]);
    std::vector<cv::Mat> Detect(cv::Mat &img_rgb, cv::Mat &img_depth, cv::Mat &img_binary);
    void OnDynamicReconfigure(DynamicReconfigureType& config, uint32_t level);
    cv::Mat rotationMatrixToEulerAngles(cv::Mat R);
    cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta);

  private:
    /// Node handler
    ros::NodeHandle nh_;
    /// ROS service
    ros::ServiceServer service_;
    /// ROS service handle <- for classification
    ros::ServiceClient classify_detections = nh_.serviceClient<classifier_pkg::classify_detections>("/classify_detections_srv");
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
    cv::Mat img_rgb_masked;

    std::vector<std::string> object_ids;
    //std::string object_ids[100] = {};

    float f_y = 574.0;
    float f_x = 574.0;

    // ORIGINAL

    // cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4239851138500654, 0.9055161821251454, -0.01664533397227496, 0.464894891647513,
    //                                                 0.9036243510870013, 0.424190730960022, 0.05937386538029141, -0.4129218758926164,
    //                                                 0.06082479228194684, 0.01013250596365761, -0.9980970278318407, 0.5500317523427357,
    //                                                 0, 0, 0, 1);


    cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4145415240703439, 0.9099628930398891, -0.01108413785169565, 0.4586872179965464,
                                                    0.9083850296878226, 0.4144934197262621, 0.05506217252054237, -0.4178745029452197,
                                                    0.05469883600672107, 0.01275689202381207, -0.9984214035393085, 0.5547044174063613,
                                                    0, 0, 0, 1);

    // cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.31420678,  0.88797765, -0.33581214,  0.53967232,
    //                                                 0.90013674,  0.39107218,  0.191876,   -0.47089768,
    //                                                 0.30170838, -0.2419881,  -0.92217884,  0.32014776,
    //                                                 0.,          0.,          0.,          1.       );




    bool service_started = false;
};