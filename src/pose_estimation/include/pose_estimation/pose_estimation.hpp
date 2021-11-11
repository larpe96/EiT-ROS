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
        std::vector<float> depth_within_perimeter(std::vector<cv::Vec3f>  circles, cv::Mat &depth_img);
        std::vector<cv::Mat> Detect(cv::Mat &img, cv::Mat &depth_img);
        void calibrate_background(cv::Mat &background_img);
        void backprojectHistogram(cv::Mat &img);
        void detect_circles(cv::Mat &img);
        std::vector<cv::Point3f> find_center_points(cv::Mat &edge_img, cv::Mat &depth_img);
        cv::Mat apply_mask(cv::Mat img);
        void show_hist(cv::MatND hist);
        std::vector<cv::Mat> convert_2_transforms(std::vector<cv::Point3f> detected_points, float img_w, float img_h);
        void drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles, cv::Scalar color, int radius);

        cv::MatND background_histogram;

    private:
        cv::Mat img;
        cv::Mat backProj;
        cv::Mat bin_image;
        cv::Mat background;

        std::vector<cv::Point3f> center_points;
        float THRESH_BACKPROJ2BIN = 30;
        int channel_numbers[1] = {0};      //Select which channel to use for histogram and backprojection (1 is HUE)
        int num_hist_bin = 180;            //Number of bin in the histogram
        float h_range[2] = { 0.0, 180.0 }; // Range of the channel used for the histogram creation
        const float* channel_ranges[1] = {h_range};
        std::vector<cv::Vec3f> debug_circles; //Stores the complete information about the circles detected in the last image.
        cv::Mat img_masked;
        cv::Rect mask_rect = cv::Rect(260, 190, 150, 200);
        float f_y = 574.0;
        float f_x = 574.0;
        cv::Mat camera2base = (cv::Mat_<float>(4,4) << 0.9185597569776569, 0.3952687376858392, 0.003255436897067616, 0.1873098683840868,
                                                      0.3733054066454857, -0.8647527818998914, -0.3359251398238113, -0.4018687658297423,
                                                     -0.1299655578620582, 0.3097825869939143, -0.9418830620437771, 0.7924941900212179,
                                                      0,0,0,1 ); //cv::Mat::eye(4, 4, CV_32F);

};
