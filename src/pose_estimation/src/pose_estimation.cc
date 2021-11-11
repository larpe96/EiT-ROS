
//#include <pose_estimation/pose_estimation.hpp>
#include <pose_estimation.hpp>
#include <ctime>
#include <fstream>

PoseEstimation::PoseEstimation()
{}

float PoseEstimation::findMedian(std::vector<float> a, int n)
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
        return (float)(a[(n - 1) / 2]
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
        return (float)a[n / 2];
    }
}

std::vector<float> PoseEstimation::depth_within_perimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img)
{
  std::vector<float> depth;
//  std::cout << contours[0] << std::endl;
  cv::Mat filledImage = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8UC1);
  cv::fillPoly(filledImage, contours, cv::Scalar(255, 255, 255));
  for (int i = 0; i < contours.size(); i++)
  {


    std::vector<cv::Point> indices;
    findNonZero(filledImage, indices);

    std::vector<float> tmp_depth;
    for(int i = 0; i < indices.size(); i++)
    {
        float depth_i = depth_img.at<float>(indices[i].y, indices[i].x);
        if(depth_i == 0)
            continue;
        tmp_depth.push_back(depth_i);
    }
    depth.push_back(findMedian(tmp_depth, tmp_depth.size()));
  }
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

std::vector<cv::Mat> PoseEstimation::Detect(cv::Mat &img_msg, cv::Mat &depth_img, cv::Mat &background_img)
{

  // Copy original image
  //std::cout << depth_img << std::endl;
  // Threshold backprojected image to create a binary mask og the projected image
  //backProj = cv::Mat::zeros(img.cols, img.rows, CV_16S);  
  cv::Mat abs_diff, diff_square, diff_norm, diff_sum;



  cv::absdiff(img_msg, this->background, abs_diff); 
  std::cout << abs_diff.type() << std::endl;
  diff_square = abs_diff.mul(abs_diff);
  //std::cout << abs_diff << std::endl;
  double min, max;
  cv::minMaxLoc(diff_square, &min, &max);
  std::cout <<"max: " << max << " min: " << min << std::endl;
  std::cout << "diff_square type"<< diff_square.type() << std::endl;
  //cv::transform(diff_square, diff_norm, cv::Mat::ones(1, 1, CV_32FC3));
  cv::Mat bgr[3];   //destination array
  cv::split(abs_diff, bgr);//split source
  diff_sum = bgr[0] + bgr[1] + bgr[2];

  std::cout << "Reaced" << std::endl;

  cv::sqrt(diff_sum, diff_norm);
  diff_norm.convertTo(diff_norm, CV_32F);
  std::cout << diff_norm.type() << "  -  " << CV_32F;
//   cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
//   cv::calcBackProject(&img, 1, this->channel_numbers, this->background_histogram, backProj, this->channel_ranges, 255.0);

  // Apply mask
  //std::cout << diff_norm << std::endl;
  img_masked = apply_mask(diff_norm);
  //cv::imshow("backproj", backProj);
  cv::imshow("masked", diff_norm);
  cv::waitKey(0); 


  // Threshold backprojected image to create a binary mask og the projected image
  backProj = cv::Mat::zeros(img_masked.cols, img_masked.rows, CV_16S);
  //cv::cvtColor(img_masked, img_masked, cv::COLOR_BGR2HSV);
  //cv::calcBackProject(&img_masked, 1, this->channel_numbers, this->background_histogram, backProj, this->channel_ranges, 255.0);
  //cv::imshow("backproj", backProj);

  // Threshold back projected image to create a binary mask og the projected image
  cv::threshold(img_masked, bin_image, this->THRESH_BACKPROJ2BIN, 255.0, cv::THRESH_BINARY);

  // Flip back projection and mask again
  //cv::bitwise_not(bin_image, bin_image);
  bin_image = apply_mask(bin_image);
  cv::imshow("binary", bin_image);

  //cv::GaussianBlur(img_masked, img_masked, cv::Size(3, 3), 0);
  //cv::imshow("masked blurred", img_masked);

  // Erode binary image
  cv::Mat structuring_element( 3, 3, CV_8U, cv::Scalar(1) );
  //cv::getStructuringElement(cv::MORPH_CROSS,(3,3));
  //cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
  for(int i = 0; i < 0; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }
  cv::imshow("Erode & dilate", bin_image);

  for(int i = 0; i < 3; i++)
  {
      cv::erode( bin_image, bin_image, structuring_element );
  }

  for(int i = 0; i < 7; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }
  cv::imshow("Erode & dilate", bin_image);

  // Filter roated_rect -> convert to Point3f representing xyz.

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  findContours( bin_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Find median depth on filled contours
  std::vector<float> depth = depth_within_perimeter(contours, depth_img);

  // Find rotated rects
  std::vector<cv::RotatedRect> rot_rect = find_rotated_rects(contours);

  for (int i = 0; i < rot_rect.size(); i++)
  {
    csv_file << id << "," << rot_rect[i].center.x <<"," <<  rot_rect[i].center.y << "," << rot_rect[i].angle << "," << rot_rect[i].size.width << "," << rot_rect[i].size.height << std::endl;
  }

  csv_file.close();

  cv::RNG rng(12345);
  std::vector<cv::RotatedRect> minRect( contours.size() );
  std::vector<cv::RotatedRect> minEllipse( contours.size() );
  for( size_t i = 0; i < contours.size(); i++ )
  {
      minRect[i] = cv::minAreaRect( contours[i] );
      if( contours[i].size() > 5 )
      {
          minEllipse[i] = cv::fitEllipse( contours[i] );
      }
  }
  cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
      // contour
      cv::drawContours( drawing, contours, (int)i, color );
      // ellipse
      cv::ellipse( drawing, minEllipse[i], color, 2 );
      // rotated rectangle
      cv::Point2f rect_points[4];
      minRect[i].points( rect_points );
      for ( int j = 0; j < 4; j++ )
      {
          line( drawing, rect_points[j], rect_points[(j+1)%4], color );
      }
  }
  imshow( "Contours", drawing );
  //center_points = find_rotated_rects(bin_image, depth_img);
  std::vector<cv::Mat> object_trans = convert_2_transforms(minRect, depth, img_msg.cols, img_msg.rows);

  return object_trans;
}

cv::Mat PoseEstimation::apply_mask(cv::Mat img)
{
  cv::Mat masked;
  cv::Mat mask = cv::Mat::zeros(img.size().height, img.size().width, CV_8U);
  mask(mask_rect) = 255;
  img.copyTo(masked, mask);
  return masked;
}

void PoseEstimation::calibrate_background(cv::Mat &background_img)
{
    this->background = background_img; 
    // // Convert image to HSV color space
    // cv::Mat background_img_HSV;
    // cv::cvtColor(background_img, background_img_HSV, cv::COLOR_BGR2HSV);
    // cv::calcHist (&background_img_HSV, 1, this->channel_numbers, cv::Mat(),
    // this->background_histogram, 1, &this->num_hist_bin, this->channel_ranges);

    // // Apply moving average on histogram
    // cv::Mat kernel_ma(3,1, CV_32F);
    // kernel_ma.at<float>(0,0) =  1.0f;
    // kernel_ma.at<float>(0,1) =  1.0f;
    // kernel_ma.at<float>(0,2) =  1.0f;
    // cv::filter2D(this->background_histogram, this->background_histogram, -1, kernel_ma);

    // // Normalize histogram
    // cv::normalize ( this->background_histogram, this->background_histogram, 1.0);
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

std::vector<cv::Mat> PoseEstimation::convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, float img_w, float img_h)
{

  std::vector<cv::Mat> trans_vec;
    for(int i = 0; i < rot_rect.size(); i++)
    {
        //cv::Point3f point = detected_points[i];
        float x = rot_rect[i].center.x - img_w/2.0;
        float y = rot_rect[i].center.y - img_h/2.0;
        cv::Mat temp_cam_2_obj = cv::Mat::eye(4, 4, CV_32F);

        float Z = depth[i]/1000; // depth_img.at<float>(point.y, point.x)/1000;
        float Y = y * Z/f_y;
        float X = x * Z/f_x;
        temp_cam_2_obj.at<float>(0, 3) = X;
        temp_cam_2_obj.at<float>(1, 3) = Y;
        temp_cam_2_obj.at<float>(2, 3) = Z;

        trans_vec.push_back((temp_cam_2_obj.inv() * camera2base).inv());
        //trans_vec.push_back(base2camera * temp_cam_2_obj);
        //std::cout << "camera2base" << camera2base << std::endl;
        //std::cout << "base2camera" << base2camera.inv() << std::endl;
        //trans_vec.push_back(temp_cam_2_obj);
    }
    return trans_vec;
}

std::vector<cv::RotatedRect> PoseEstimation::find_rotated_rects(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::RotatedRect> rot_rect;
    for(int i = 0; i < contours.size(); i++)
    {
        rot_rect.push_back(cv::minAreaRect(contours[i]));
    }

    return rot_rect;
}
