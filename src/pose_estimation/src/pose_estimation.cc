#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation()
{}

void PoseEstimation::drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles, cv::Scalar color, int radius)
{
    for(int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int rad = cvRound(circles[i][2]);
        cv::circle(img, center, radius, color, -1, 8, 0 );
        cv::circle(img, center, rad, cv::Scalar(255,0,0), radius, 8, 0 );
    }
}

std::vector<cv::Mat> PoseEstimation::Detect(cv::Mat &img_msg, cv::Mat &depth_img)
{
  // Copy original image
  img_msg.copyTo(img);
  //std::cout << depth_img << std::endl;
  // Threshold backprojected image to create a binary mask og the projected image
  backProj = cv::Mat::zeros(img.cols, img.rows, CV_16S);
  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  cv::calcBackProject(&img, 1, this->channel_numbers, this->background_histogram, backProj, this->channel_ranges, 255.0);

  // Apply mask
  img_masked = apply_mask(backProj);
  //cv::imshow("backproj", backProj);
  cv::imshow("masked", img_masked);

  //cv::GaussianBlur(img_masked, img_masked, cv::Size(3, 3), 0);
  //cv::imshow("masked blurred", img_masked);


  // Threshold backprojected image to create a binary mask og the projected image
  cv::threshold(img_masked, bin_image, this->THRESH_BACKPROJ2BIN, 255.0, cv::THRESH_BINARY);
  cv::bitwise_not(bin_image, bin_image);
  cv::imshow("bin img w/o erode", bin_image);
  // Erode binary image
  cv::Mat structuring_element( 3, 3, CV_8U, cv::Scalar(1) );
  for(int i = 0; i < 2; i++)
  {
      cv::erode( bin_image, bin_image, structuring_element );
  }

  for(int i = 0; i < 3; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }

  cv::imshow("bin img", bin_image);
  center_points = find_center_points(bin_image);

  std::vector<cv::Mat> object_trans = convert_2_transforms(center_points, depth_img, img_msg.cols, img_msg.rows);

  return object_trans;
}

cv::Mat PoseEstimation::apply_mask(cv::Mat img)
{
  cv::Mat masked;
  cv::Mat mask = cv::Mat::zeros(img.size().height, img.size().width, CV_8U);
  mask(mask_rect) = 255;
  img.copyTo(masked, mask);
  //cv::imshow("masked", masked);
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
}

std::vector<cv::Mat> PoseEstimation::convert_2_transforms(std::vector<cv::Point2f> detected_points, cv::Mat depth_img, double img_w, double img_h)
{
  std::vector<cv::Mat> trans_vec;
    for(int i = 0; i < detected_points.size(); i++)
    {
        cv::Point2f point = detected_points[i];
        double x = point.x - img_w/2.0;
        double y = point.y - img_h/2.0;
        cv::Mat temp_cam_2_obj = cv::Mat::eye(4, 4, CV_32F);
        std::cout << "Z: " << depth_img.at<_Float32>(point.y, point.x) << std::endl;
        _Float32 Z = depth_img.at<_Float32>(point.y, point.x)/1000.0;
        _Float32 Y = y * Z/f_y;
        _Float32 X = x * Z/f_x;
        temp_cam_2_obj.at<_Float32>(0, 3) = X;
        temp_cam_2_obj.at<_Float32>(1, 3) = Y;
        temp_cam_2_obj.at<_Float32>(2, 3) = Z;
        trans_vec.push_back(base_2_camera * temp_cam_2_obj);
    }
    return trans_vec;
}

std::vector<cv::Point2f> PoseEstimation::find_center_points(cv::Mat &edge_img)
{
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point2f> centerPoints;
    cv::HoughCircles(edge_img, circles, cv::HOUGH_GRADIENT, 1, edge_img.rows/16, 5, 5, 5, 40);
/*
    cv::RNG rng(12345);
    int thresh = 100;
    cv::Mat canny_output;
    cv::Canny( edge_img, canny_output, thresh, thresh*2 );

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
    */
    cv::Mat tmp_img;
    cv::cvtColor(edge_img, tmp_img, cv::COLOR_GRAY2BGR);
    drawCircles(tmp_img, circles, cv::Scalar(255, 255, 0), 3);

    cv::imshow("edge_img circles", tmp_img);

    this->debug_circles = circles;
    for( size_t i = 0; i < circles.size(); i++ )
    {
         cv::Point2f center(circles[i][0], circles[i][1]);
         centerPoints.push_back(center);
    }
    return centerPoints;
}

