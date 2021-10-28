#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation()
{}

std::vector<cv::Point2f> PoseEstimation::Detect(cv::Mat &img_msg)
{
  // Copy original image
  img_msg.copyTo(img);

  // Threshold backprojected image to create a binary mask og the projected image
  backProj = cv::Mat::zeros(img.cols, img.rows, CV_16S);
  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  cv::calcBackProject(&img, 1, this->channel_numbers, this->background_histogram, backProj, this->channel_ranges, 255.0);

  // Apply mask
  img_masked = apply_mask(backProj);
  cv::imshow("backproj", backProj);

  // Threshold backprojected image to create a binary mask og the projected image
  cv::threshold(img_masked, bin_image, this->THRESH_BACKPROJ2BIN, 255.0, cv::THRESH_BINARY);
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

  center_points = find_center_points(bin_image);
  return center_points;
}

cv::Mat PoseEstimation::apply_mask(cv::Mat img)
{
  cv::Mat masked;
  cv::Mat mask = cv::Mat::zeros(img.size().height, img.size().width, CV_8U);
  mask(mask_rect) = 1;
  img.copyTo(masked, mask);
  cv::imshow("masked", masked);
  return masked;
}

void PoseEstimation::calibrate_background(cv::Mat &background_img)
{
    // Convert image to HSV color space
    cv::Mat background_img_HSV;
    cv::cvtColor(background_img, background_img_HSV, cv::COLOR_BGR2HSV);
    cv::calcHist (&background_img_HSV, 1, this->channel_numbers, cv::Mat(),
    this->background_histogram, 1, &this->num_hist_bin, this->channel_ranges);

    // Apply moving average on histogram
    cv::Mat kernel_ma(3,1, CV_32F);
    kernel_ma.at<float>(0,0) =  1.0f;
    kernel_ma.at<float>(0,1) =  1.0f;
    kernel_ma.at<float>(0,2) =  1.0f;
    cv::filter2D(this->background_histogram, this->background_histogram, -1, kernel_ma);

    // Normalize histogram
    cv::normalize ( this->background_histogram, this->background_histogram, 1.0);
}

void PoseEstimation::show_hist(cv::MatND hist)
{
    //create an 8 bits single channel image to hold the histogram
    //paint it white
    cv::Mat imgHistogram = cv::Mat(cv::Size(num_hist_bin, 50), CV_8UC3, cv::Scalar(0,0,0));
    cv::rectangle(imgHistogram, cv::Point(0,0), cv::Point(256,50), CV_RGB(255,255,255), -1);
    float value;
    int normalized;

    //draw the histogram
    for(int i=0; i < num_hist_bin; i++)
    {
            //value = cv::QueryHistValue_1D( this->background_histogram, i);
            value = hist.at<float>(0,i);
            normalized = cvRound(value*50/h_range[1]);
            cv::line(imgHistogram, cv::Point(i,50), cv::Point(i,50-normalized), CV_RGB(0,0,0));
    }

  cv::imshow("filtered histogram", imgHistogram);
  cv::waitKey(0);
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