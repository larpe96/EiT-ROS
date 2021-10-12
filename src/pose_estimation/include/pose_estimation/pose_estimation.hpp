#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <vector>

class PoseEstimation
{
    public:
        PoseEstimation();
        std::vector<cv::Point2f> Detect(cv::Mat &img);
        void calibrate_background(cv::Mat &background_img);
        void backprojectHistogram(cv::Mat &img);
        void detect_circles(cv::Mat &img);
        std::vector<cv::Point2f> find_center_points(cv::Mat &edge_img);

    private:
        cv::Mat img;
        cv::Mat backProj;
        cv::Mat bin_image;
        std::vector<cv::Point2f> center_points;

        cv::MatND background_histogram;
        float THRESH_BACKPROJ2BIN = 3;
        int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
        int num_hist_bin = 180;            //Number of bin in the histogram
        float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
        const float* channel_ranges[1] = {h_range};
        std::vector<cv::Vec3f> debug_circles; //Stores the complete information about the circles detected in the last image.

};
