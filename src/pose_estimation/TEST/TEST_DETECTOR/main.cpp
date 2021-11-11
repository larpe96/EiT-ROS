#include <iostream>
#include "pose_estimation.hpp"
#include <opencv2/opencv.hpp>
using namespace std;

int main()
{

    cout << "Hello World!" << endl;
    cv::Mat background = cv::imread("background.jpg");
    cv::Mat img  = cv::imread("img.jpg");
    cv::Mat img_depth;
    PoseEstimation detector;
    detector.calibrate_background(background);
    detector.Detect(img, img_depth);
    return 0;
}
