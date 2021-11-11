#include <iostream>
#include "pose_estimation.hpp"
#include <opencv2/opencv.hpp>
using namespace std;

int main()
{

    cout << "Hello World!" << CV_32FC3 << endl;
    cv::Mat background_tmp = cv::imread("../TEST_DETECTOR/background.jpg", cv::IMREAD_COLOR);
    cv::Mat img  = cv::imread("../TEST_DETECTOR//img.jpg", cv::IMREAD_COLOR);
    cv::Mat img_depth = cv::Mat::ones(img.rows, img.cols, CV_32FC1);
    cv::imshow("background", background_tmp);
    cv::imshow("img", img);
    cv::waitKey(0);


    img.convertTo(img, CV_32FC3);
    cv::Mat background;
    background_tmp.convertTo(background, CV_32FC3);

    PoseEstimation detector;
    std::cout<< "background type: "<< background.type() << std::endl;

    detector.calibrate_background(background);
    detector.Detect(img, img_depth);

    return 0;
}
