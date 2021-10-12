#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation()
{}

std::vector<cv::Point2f> PoseEstimation::Detect(cv::Mat &img_msg)
{
  // Copy original image
  img_msg.copyTo(img);

  // Crop image
  img = img(cv::Range(155,314), cv::Range(205,318));

  backProj = cv::Mat::zeros(img.cols, img.rows, CV_16S);

  // Threshold backprojected image to create a binary mask og the projected image
  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  cv::calcBackProject(&img, 1, this->channel_numbers, this->background_histogram, backProj, this->channel_ranges, 255.0);

  cv::imshow("backproj", backProj);

  // Threshold backprojected image to create a binary mask og the projected image
  cv::threshold(backProj, bin_image, this->THRESH_BACKPROJ2BIN, 255.0, cv::THRESH_BINARY);
  cv::bitwise_not(bin_image, bin_image);

  // Erode binary image
  cv::Mat structuring_element( 3, 3, CV_8U, cv::Scalar(1) );
  for(int i = 0; i < 3; i++)
  {
      cv::erode( bin_image, bin_image, structuring_element );
  }

  for(int i = 0; i < 3; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }

  cv::waitKey(1);

  center_points = find_center_points(bin_image);
  return center_points;
}

void PoseEstimation::calibrate_background(cv::Mat &background_img)
{
    cv::Mat background_img_HSV;
    cv::cvtColor(background_img, background_img_HSV, cv::COLOR_BGR2HSV);

    cv::calcHist (&background_img_HSV, 1, this->channel_numbers, cv::Mat(),
    this->background_histogram, 1, &this->num_hist_bin, this->channel_ranges);

    cv::normalize ( this->background_histogram, this->background_histogram, 1.0);
}

std::vector<cv::Point2f> PoseEstimation::find_center_points(cv::Mat &edge_img)
{
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point2f> centerPoints;
    cv::HoughCircles(edge_img, circles, cv::HOUGH_GRADIENT, 1, edge_img.rows/16, 100, 15, 10, 30);

    this->debug_circles = circles;
    for( size_t i = 0; i < circles.size(); i++ )
    {
         cv::Point2f center(circles[i][0], circles[i][1]);
         centerPoints.push_back(center);
    }
    return centerPoints;
}