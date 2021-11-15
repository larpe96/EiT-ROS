
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
        // on n/2th indexf
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

std::vector<cv::Mat> PoseEstimation::Detect(cv::Mat &img_msg, cv::Mat &depth_img)
{
  
  
  img_msg.convertTo(img_msg, CV_32FC3);
  cv::Mat abs_diff, diff_square, diff_norm, diff_sum;

  cv::absdiff(img_msg, this->background, abs_diff); 
  diff_square = abs_diff.mul(abs_diff);

  // Sum across channels
  cv::Mat bgr[3];   //destination array
  cv::split(abs_diff, bgr);//split source
  diff_sum = bgr[0] + bgr[1] + bgr[2];

  // Elementswise squareroot.
  cv::sqrt(diff_sum, diff_norm);

  diff_norm.convertTo(diff_norm, CV_8UC1);
  cv::imshow("diff_norm", diff_norm);

  img_masked = apply_mask(diff_norm);

  cv::imshow("masked", img_masked);
  
  // Threshold back projected image to create a binary mask og the projected image
  cv::threshold(img_masked, bin_image, this->THRESH_BACKPROJ2BIN, 255.0, cv::THRESH_BINARY);

  cv::Mat structuring_element( 3, 3, CV_8U, cv::Scalar(1) );

  cv::imshow("Erode & dilate", bin_image);

  for(int i = 0; i < 1; i++)
  {
      cv::erode( bin_image, bin_image, structuring_element );
  }

  for(int i = 0; i < 1; i++)
  {
      cv::dilate(bin_image, bin_image, structuring_element );
  }

  bin_image.convertTo(bin_image, CV_8U);

  // Filter roated_rect -> convert to Point3f representing xyz.

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> init_contours;
  cv::findContours( bin_image, init_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  //cv::findContours( bin_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::Mat filledImage = cv::Mat::zeros(img_msg.rows, img_msg.cols, CV_8UC1);
  //cv::fillPoly(filledImage, init_contours, cv::Scalar(255));
  for (int i = 0; i < (int)init_contours.size(); i++)
  {
    cv::drawContours( filledImage, init_contours, i, cv::Scalar(255) , CV_FILLED);    
  }
  cv::findContours( filledImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  
  cv::imshow("filled image", filledImage);

  // Find median depth on filled contours
  std::vector<float> depth = depth_within_perimeter(contours, depth_img);

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
  cv::Mat drawing = img_msg.clone(); //cv::Mat::zeros( img_msg.size(), CV_8UC3 );
  drawing.convertTo(drawing, CV_8UC3);

  for( size_t i = 0; i< contours.size(); i++ )
  {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
      // contour
      cv::drawContours( drawing, contours, (int)i, color );
      //cv::drawContours( drawing, contours, (int)i, color , CV_FILLED);    
      // ellipse
      cv::ellipse( drawing, minEllipse[i], color, 2 );
      // rotated rectangle
      cv::Point2f rect_points[4];
      minRect[i].points( rect_points );
      for ( int j = 0; j < 4; j++ )
      {
          cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color );
      }
  }
  cv::imshow( "Contours", drawing );
//  cv::waitKey(0);
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
    this->background.convertTo(this->background, CV_32FC3);
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
