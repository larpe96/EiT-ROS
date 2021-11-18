#include <pose_estimation/detector.h>

/*#include <vector>
#include <string>
#include <opencv/highgui.h>
#include <ctime>
#include <fstream>
*/

namespace detector
{
  cv::Mat FillContours(cv::Mat img, std::vector<std::vector<cv::Point>> contours)
  {
    cv::Mat filledImage = cv::Mat::zeros(img.rows, img.cols, img.type());
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( filledImage, contours,  (int)i, cv::Scalar(255), cv::FILLED);
    }

    return filledImage;
  }

  std::vector<float> DepthWithinPerimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img)
  {
    std::vector<float> depth_within;
    cv::Mat filledImage = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8UC1);
    //cv::fillPoly(filledImage, contours, cv::Scalar(255, 255, 255));
    filledImage = FillContours(filledImage, contours);

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
      depth_within.push_back(FindMedian(tmp_depth, tmp_depth.size()));
    }
    return depth_within;
  }

  float FindMedian(std::vector<float> a, int n)
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

  std::vector<cv::RotatedRect> FindRotatedRects(std::vector<std::vector<cv::Point>> contours)
  {
      std::vector<cv::RotatedRect> rot_rect;
      for(int i = 0; i < contours.size(); i++)
      {
          rot_rect.push_back(cv::minAreaRect(contours[i]));
      }
      return rot_rect;
  }

  void SaveToFile(cv::Mat img_rgb, cv::Mat img_depth, std::vector<cv::RotatedRect> rot_rects)
  {
    std::ofstream csv_file("data/data.csv", std::ofstream::out | std::ofstream::app);

    // ID for csv file
    time_t current_time;
    std::string id;
    std::string id_path;
    current_time = time(NULL);
    id = std::to_string(current_time);
    id_path = "data/" + id;

    // Save images
    cv::imwrite(id_path + "orig_rgb.png", img_rgb);
    cv::imwrite(id_path + "orig_depth.png", img_depth);
    cv::imwrite("backgroundEmpty.png", img_rgb);

    // Write to CSV-file
    for (int i = 0; i < rot_rects.size(); i++)
    {
      csv_file << id << "," << rot_rects[i].center.x <<"," <<  rot_rects[i].center.y << "," << rot_rects[i].angle << "," << rot_rects[i].size.width << "," << rot_rects[i].size.height << std::endl;
    }
    csv_file.close();
  }

  void ShowDetections(cv::Mat img_rgb, std::vector<std::vector<cv::Point>> contours, std::vector<cv::RotatedRect> rot_rects)
  {
    cv::RNG rng = rng(12345);
    //std::vector<cv::RotatedRect> minRect( contours.size() );
    std::vector<cv::RotatedRect> minEllipse( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        //minRect[i] = cv::minAreaRect( contours[i] );
        if( contours[i].size() > 5 )
        {
            minEllipse[i] = cv::fitEllipse( contours[i] );
        }
    }

    img_rgb.convertTo(img_rgb, CV_8UC3);
    cv::Mat drawing = cv::Mat::zeros( img_rgb.size(), CV_8UC3 );
    img_rgb.copyTo(drawing);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        // contour
        cv::drawContours( drawing, contours, (int)i, color );

        // ellipse
        //cv::ellipse( drawing, minEllipse[i], color, 2 );
        // rotated rectangle
        cv::Point2f rect_points[4];
        //minRect[i].points( rect_points );
        rot_rects[i].points( rect_points );
        for ( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color );
        }
    }
    cv::imshow( "Contours", drawing );
  }

  std::vector<cv::Mat> convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, int img_w, int img_h, float f_x, float f_y, cv::Mat camera2base)
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

        float angle = rot_rect[i].angle * 2*M_PI/360;
        angle += 90 * 2*M_PI/360;

        cv::Mat R_z = (cv::Mat_ <float>(3,3) << cos(angle), -sin(angle), 0,
                                                sin(angle), cos(angle), 0,
                                                0, 0, 1);
        
        R_z.copyTo(temp_cam_2_obj(cv::Rect(0, 0, R_z.cols, R_z.rows)));

        trans_vec.push_back((temp_cam_2_obj.inv() * camera2base).inv());
      }
      return trans_vec;
  }

  cv::Mat ApplyMask(cv::Mat img, const int& x, const int& y, const int& width, const int& height)
  {
    cv::Mat masked;
    cv::Mat mask = cv::Mat::zeros(img.size().height*2, img.size().width*2, CV_8U);
    cv::Rect mask_rect = cv::Rect(x, y, width, height);
    mask(mask_rect) = 255;
    img.copyTo(masked, mask(cv::Rect(0, 0, img.size().width, img.size().height)));
    return masked;
  }

  cv::Mat ErodeAndDilate(cv::Mat img, int erodeSize, int dilateSize)
  {
    cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));

    for(int i = 0; i < erodeSize; i++)
    {
        cv::erode( img, img, structuring_element );
    }

    for(int i = 0; i < dilateSize; i++)
    {
        cv::dilate(img, img, structuring_element );
    }

    return img;
  }

  cv::Mat DiffNorm(cv::Mat &img_msg, cv::Mat background)
  {
    img_msg.convertTo(img_msg, CV_32FC3);
    cv::Mat abs_diff, diff_square, diff_norm, diff_sum;
    background.convertTo(background, CV_32FC3);

    cv::absdiff(img_msg, background, abs_diff); 
    diff_square = abs_diff.mul(abs_diff);

    // Sum across channels
    cv::Mat bgr[3];   //destination array
    cv::split(abs_diff, bgr);//split source
    diff_sum = bgr[0] + bgr[1] + bgr[2];

    // Elementswise squareroot.
    cv::sqrt(diff_sum, diff_norm);

    diff_norm.convertTo(diff_norm, CV_8UC1);
    return diff_norm;
  }

} // namespace detector



