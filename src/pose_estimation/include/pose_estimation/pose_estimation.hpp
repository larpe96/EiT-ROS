#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <vector>

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
        std::vector<cv::RotatedRect> find_rotated_rects(  std::vector<std::vector<cv::Point>> contours);
        cv::Mat apply_mask(cv::Mat img);
        void show_hist(cv::MatND hist);
        std::vector<cv::Mat> convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, float img_w, float img_h);
        void drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles, cv::Scalar color, int radius);

        cv::MatND background_histogram;

    private:
        cv::Mat img;
        cv::Mat backProj;
        cv::Mat bin_image;

        std::vector<cv::Point3f> center_points;
        float THRESH_BACKPROJ2BIN = 30;
        int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
        int num_hist_bin = 180;            //Number of bin in the histogram
        float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
        const float* channel_ranges[1] = {h_range};
        std::vector<cv::Vec3f> debug_circles; //Stores the complete information about the circles detected in the last image.
        cv::Mat img_masked;
        cv::Rect mask_rect = cv::Rect(200, 185, 200, 155);
        float f_y = 574.0;
        float f_x = 574.0;
        cv::Mat camera2base = (cv::Mat_<float>(4,4) << -0.4092872843859879, 0.9119913122596123, -0.02749118409475548, 0.4613890083554278,
 0.9118304765329119, 0.4099132124339962, 0.02315902281622022, -0.396963379231898,
 0.03238982719471067, -0.01558860593609109, -0.9993537384026068, 0.5628008106183637,
 0, 0, 0, 1);

        cv::Mat base2camera = (cv::Mat_<float>(4,4) << -0.3200615474180835, 0.906443881953242, 0.2755360134971088, 0.5131311099627459,
                                                        0.9247229143918688, 0.3621450629584728, -0.1172112834731024, -0.2413966376231433,
                                                        -0.2060294577553179, 0.2172796406577029, -0.9541181374927892, 0.5905108421276393,
                                                        0, 0, 0, 1);
};