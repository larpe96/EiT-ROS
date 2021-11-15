#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation(): nh_("~"), dynamic_reconfigure_server_(nh_)
{}

void PoseEstimation::Initialize(const ros::NodeHandle& nh)
{
  // Node handler
  nh_ = nh;
  // Dynamic reconfigure
  dynamic_reconfigure::Server<DynamicReconfigureType>::CallbackType f;
  f = boost::bind(&PoseEstimation::OnDynamicReconfigure, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);
}

void PoseEstimation::OnImage(const sensor_msgs::ImageConstPtr& img_rgb_msg, const sensor_msgs::ImageConstPtr& img_depth_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_rgb_msg, sensor_msgs::image_encodings::BGR8);
		cv_ptr_depth = cv_bridge::toCvCopy(img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

  // Copy images
  cv_ptr->image.copyTo(img_rgb);
  cv_ptr_depth->image.copyTo(img_depth);

  // Get diff image
  cv::absdiff(img_rgb, img_empty_background, img_diff);

  // Apply mask
  img_diff_masked = apply_mask(img_diff);

  // Grey
  cv::cvtColor(img_diff_masked, img_diff_masked_grey, cv::COLOR_BGR2GRAY);

  // Threshold back projected image to create a binary mask og the projected image
  cv::threshold(img_diff_masked_grey, img_binary, config_.threshold_binary, 255.0, cv::THRESH_BINARY);

  // Erode binary image
  //cv::Mat structuring_element( 3, 3, CV_8U, cv::Scalar(1));
  cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));

  for(int i = 0; i < config_.erosion_size; i++)
  {
      cv::erode( img_binary, img_binary, structuring_element );
  }

  for(int i = 0; i < config_.dilation_size; i++)
  {
      cv::dilate(img_binary, img_binary, structuring_element );
  }


  // Show images
	cv::imshow("rgb", img_rgb);
	cv::imshow("depth", img_depth);

	cv::imshow("diff", img_diff);
  cv::imshow("diff masked", img_diff_masked);
  cv::imshow("binary", img_binary);
  cv::waitKey(1);
}

void PoseEstimation::calibrate_background(std::string background_path)
{
  img_background = cv::imread(background_path);
  // Convert image to HSV color space
  cv::cvtColor(img_background, background_img_HSV, cv::COLOR_BGR2HSV);
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

bool PoseEstimation::Estimate_pose(pose_estimation::pose_est_srv::Request   &req,
		                pose_estimation::pose_est_srv::Response  &res)
{
	object_points = Detect(img_rgb, img_depth);
	posearray.header.stamp = ros::Time::now();

	for(int i = 0; i < object_points.size(); i++)
	{
		cv::Mat tmp_trans = object_points[i];
		float quat[4];
		getQuaternion(tmp_trans(cv::Rect(0,0,3,3)), quat);

		geometry_msgs::Pose p;
		p.position.x = tmp_trans.at<float>(0, 3);
		p.position.y = tmp_trans.at<float>(1, 3);
		p.position.z = tmp_trans.at<float>(2, 3);
    p.orientation.x = quat[0];
		p.orientation.y = quat[1];
		p.orientation.z = quat[2];
		p.orientation.w = quat[3];

		posearray.poses.push_back(p);
		//std::cout << tmp_trans << std::endl;
	}
	res.rel_object_poses = posearray;

	out_str = "Number of detected objects: " + std::to_string(object_points.size());
	ROS_INFO_STREAM(out_str);
	return true;
}

void PoseEstimation::getQuaternion(cv::Mat R, float Q[])
{
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);

    if (trace > 0.0)
    {
        float s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    }

    else
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        float s = sqrt(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}

std::vector<cv::Mat> PoseEstimation::Detect(cv::Mat &img_rgb, cv::Mat &img_depth)
{
  std::ofstream csv_file("data/data.csv", std::ofstream::out | std::ofstream::app);

  // ID for csv file
  current_time = time(NULL);
  id = std::to_string(current_time);
  id_path = "data/" + id;

  // Save images
  cv::imwrite(id_path + "orig_rgb.png", img_rgb);
  cv::imwrite(id_path + "orig_depth.png", img_depth);
  cv::imwrite("backgroundEmpty.png", img_rgb);

  // Find contours
  findContours(img_binary, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Draw contours
  rng = rng(12345);
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

  cv::Mat filledImage = cv::Mat::zeros(img_rgb.rows, img_rgb.cols, CV_8UC1);
  cv::Mat drawing = cv::Mat::zeros( img_rgb.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
      // contour
      cv::drawContours( drawing, contours, (int)i, color );
      cv::drawContours( filledImage, contours,  (int)i, cv::Scalar(255), cv::FILLED);
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
  cv::imshow( "Contours", drawing );
  cv::imshow("filled", filledImage);

  // Find contours
  std::vector<std::vector<cv::Point>> contours2;
  findContours(filledImage, contours2, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


  // Find median depth on filled contours
  depth = depth_within_perimeter(contours2, img_depth);

  // Find rotated rects
  rot_rects = find_rotated_rects(contours2);

  // Write to CSV-file
  for (int i = 0; i < rot_rects.size(); i++)
  {
    csv_file << id << "," << rot_rects[i].center.x <<"," <<  rot_rects[i].center.y << "," << rot_rects[i].angle << "," << rot_rects[i].size.width << "," << rot_rects[i].size.height << std::endl;
  }
  csv_file.close();

  // Draw contours
  rng = rng(12345);
  std::vector<cv::RotatedRect> minRect2( contours2.size() );
  std::vector<cv::RotatedRect> minEllipse2( contours2.size() );
  for( size_t i = 0; i < contours2.size(); i++ )
  {
      minRect2[i] = cv::minAreaRect( contours2[i] );
      if( contours2[i].size() > 5 )
      {
          minEllipse2[i] = cv::fitEllipse( contours2[i] );
      }
  }

  cv::Mat filledImage2 = cv::Mat::zeros(img_rgb.rows, img_rgb.cols, CV_8UC1);
  cv::Mat drawing2 = cv::Mat::zeros( img_rgb.size(), CV_8UC3 );
  for( size_t i = 0; i< contours2.size(); i++ )
  {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
      // contour
      cv::drawContours( drawing2, contours2, (int)i, color );
      cv::drawContours( filledImage2, contours2,  (int)i, cv::Scalar(255), cv::FILLED);
      // ellipse
      cv::ellipse( drawing2, minEllipse2[i], color, 2 );
      // rotated rectangle
      cv::Point2f rect_points[4];
      minRect2[i].points( rect_points );
      for ( int j = 0; j < 4; j++ )
      {
          line( drawing2, rect_points[j], rect_points[(j+1)%4], color );
      }
  }
  cv::imshow( "Contours2", drawing2 );
  cv::imshow("filled2", filledImage2);


  std::vector<cv::Mat> object_trans = convert_2_transforms(rot_rects, depth, img_rgb.size().width, img_rgb.size().height);

  return object_trans;
}

cv::Mat PoseEstimation::apply_mask(cv::Mat img)
{
  cv::Rect mask_rect = cv::Rect(config_.mask_x, config_.mask_y, config_.mask_w, config_.mask_h);
  cv::Mat masked;
  cv::Mat mask = cv::Mat::zeros(img.size().height, img.size().width, CV_8U);
  mask(mask_rect) = 255;
  img.copyTo(masked, mask);
  return masked;
}

std::vector<float> PoseEstimation::depth_within_perimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img)
{
  std::vector<float> depth_within;
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
    depth_within.push_back(findMedian(tmp_depth, tmp_depth.size()));
  }
  return depth_within;
}

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

std::vector<cv::RotatedRect> PoseEstimation::find_rotated_rects(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::RotatedRect> rot_rect;
    for(int i = 0; i < contours.size(); i++)
    {
        rot_rect.push_back(cv::minAreaRect(contours[i]));
    }
    return rot_rect;
}

std::vector<cv::Mat> PoseEstimation::convert_2_transforms(std::vector<cv::RotatedRect> rot_rect, std::vector<float> depth, int img_w, int img_h)
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
    }
    return trans_vec;
}

void PoseEstimation::OnDynamicReconfigure(PoseEstimation::DynamicReconfigureType &config, uint32_t level)
{
  (void)level;
  config_ = config;
  // Calibrate background
  //calibrate_background(config_.background_path);
  img_empty_background = cv::imread(config_.empty_img_path);
  std::cout << "reconfig" << std::endl;


  // Subscribers

  bool resubscribe = !camera_sync_ ||
  subscriber_rgb_->getTopic() != config_.input_topic_rgb ||
  subscriber_depth_->getTopic() != config_.input_topic_depth;
  if (resubscribe)
  {
    subscriber_rgb_ = std::make_shared<decltype(subscriber_rgb_)::element_type>(nh_, config_.input_topic_rgb, 1);
    subscriber_depth_ = std::make_shared<decltype(subscriber_depth_)::element_type>(nh_, config_.input_topic_depth, 1);

    camera_sync_ = std::make_shared<CameraSynchronizer>(CameraSyncPolicy(1),
            *subscriber_rgb_,
            *subscriber_depth_
          );

  camera_sync_->registerCallback(boost::bind(&PoseEstimation::OnImage, this, _1, _2));


  }

  // Service
  if(!service_started)
  {
    service_ = nh_.advertiseService(config_.output_topic, &PoseEstimation::Estimate_pose, this);
    ROS_INFO("Ready to estimate relative position of the object.");
    service_started = true;
  }
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
