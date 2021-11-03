#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation()
{}

double PoseEstimation::findMedian(std::vector<_Float32> a, int n)
{
    if(n == 1)
    {
        return a[0];
    }
    // If size of the arr[] is even
    if (n % 2 == 0) {

        // Applying nth_element
        // on n/2th index
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());

        // Applying nth_element
        // on (n-1)/2 th index
        nth_element(a.begin(),
                    a.begin() + (n - 1) / 2,
                    a.end());

        // Find the average of value at
        // index N/2 and (N-1)/2
        return (_Float32)(a[(n - 1) / 2]
                        + a[n / 2])
               / 2.0;
    }

    // If size of the arr[] is odd
    else {

        // Applying nth_element
        // on n/2
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());

        // Value at index (N/2)th
        // is the median
        return (_Float32)a[n / 2];
    }
}

std::vector<_Float32> PoseEstimation::depth_within_perimeter(std::vector<cv::Vec3f>  circles, cv::Mat &depth_img)
{
    std::vector<_Float32> depth;
    for(int i = 0; i < circles.size(); i++)
    {
        cv::Mat background = cv::Mat::zeros(cv::Size(depth_img.cols, depth_img.rows), CV_8U);
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(background, center, radius, 255, -1, cv::LINE_8, 0);
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( background, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        std::vector<_Float32> tmp_depth;
        cv::imshow("debug", background);
        cv::waitKey(0);
        std::cout << contours[0].size() << std::endl;
        for(int i = 0; i < contours[0].size(); i++)
        {
            _Float32 depth_i = depth_img.at<_Float32>(contours[0][i].y, contours[0][i].x);
            if(depth_i == 0)
                continue;
            tmp_depth.push_back(depth_i);
        }
        depth.push_back(findMedian(tmp_depth, tmp_depth.size()));
    }
    std::cout << depth.size() << std::endl;
    return depth;


}

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
  for(int i = 0; i < 1; i++)
  {
      cv::erode( bin_image, bin_image, structuring_element );
  }

  for(int i = 0; i < 3; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }

  cv::imshow("bin img", bin_image);
  center_points = find_center_points(bin_image, depth_img);

  std::vector<cv::Mat> object_trans = convert_2_transforms(center_points, img_msg.cols, img_msg.rows);

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

std::vector<cv::Mat> PoseEstimation::convert_2_transforms(std::vector<cv::Point3f> detected_points, double img_w, double img_h)
{
  std::vector<cv::Mat> trans_vec;
    for(int i = 0; i < detected_points.size(); i++)
    {
        cv::Point3f point = detected_points[i]; 
        double x = point.x - img_w/2.0; 
        double y = point.y - img_h/2.0; 
        cv::Mat temp_cam_2_obj = cv::Mat::eye(4, 4, CV_32F); 
        double Z = point.z/1000; // depth_img.at<double>(point.y, point.x)/1000;
        double Y = y * Z/f_y; 
        double X = x * Z/f_x;
        temp_cam_2_obj.at<double>(0, 3) = X; 
        temp_cam_2_obj.at<double>(1, 3) = Y; 
        temp_cam_2_obj.at<double>(2, 3) = Z; 

        trans_vec.push_back(base_2_camera * temp_cam_2_obj);
    }
    return trans_vec;
}


std::vector<cv::Point3f> PoseEstimation::find_center_points(cv::Mat &edge_img, cv::Mat &depth_img)
{
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point3f> centerPoints;
    cv::HoughCircles(edge_img, circles, cv::HOUGH_GRADIENT, 1, edge_img.rows/16, 80, 15, 10, 30);
    cv::Mat tmp_img; 
    cv::cvtColor(edge_img, tmp_img, cv::COLOR_GRAY2BGR); 
    drawCircles(tmp_img, circles, cv::Scalar(255, 255, 0), 3);
    
    cv::imshow("edge_img circles", tmp_img); 
    std::vector<_Float32> depth = depth_within_perimeter(circles, depth_img); 
    this->debug_circles = circles;
    for( size_t i = 0; i < circles.size(); i++ )
    {
         cv::Point3f center(circles[i][0], circles[i][1], depth[i]);
         centerPoints.push_back(center);
    }
    return centerPoints;
}