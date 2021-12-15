#include <pose_estimation/pose_estimation.hpp>

PoseEstimation::PoseEstimation(): nh_("~"), dynamic_reconfigure_server_(nh_)
{
}

void PoseEstimation::Initialize(const ros::NodeHandle& nh)
{
  // Node handler
  nh_ = nh;
  // Dynamic reconfigure
  dynamic_reconfigure::Server<DynamicReconfigureType>::CallbackType f;
  f = boost::bind(&PoseEstimation::OnDynamicReconfigure, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);
  sub_groundTruth= nh_.subscribe("/GT_rects", 20, &PoseEstimation::compareToGT, this);
}

void PoseEstimation::setFiles()
{
  poseDataFile.open("/home/user/workspace/pose_dataV4.csv",std::fstream::out);// | std::fstream::app);
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
  bool use_DiffNorm = 1;
  if(use_DiffNorm)
  {
    img_diff = detector::DiffNorm(img_rgb, img_empty_background);
  }
  else
  {
    cv::absdiff(img_rgb, img_empty_background, img_diff);
    
    // Grey
    cv::cvtColor(img_diff, img_diff, cv::COLOR_BGR2GRAY);
  }

  // Apply mask
  img_diff_masked = detector::ApplyMask(img_diff, config_.mask_x, config_.mask_y, config_.mask_w, config_.mask_h);
  rgb_masked = detector::ApplyMask(img_rgb, config_.mask_x, config_.mask_y, config_.mask_w, config_.mask_h);

  // Threshold back projected image to create a binary mask og the projected image
  cv::threshold(img_diff_masked, img_binary, config_.threshold_binary, 255.0, cv::THRESH_BINARY);

  // Erode binary image
  img_binary = detector::ErodeAndDilate(img_binary, config_.erosion_size, config_.dilation_size);

  // Image for testing depth registration
  double max = 1;
  double min = 0;
  cv::Mat falseColorMap;
  cv::Mat adjMap;
  cv::minMaxIdx(img_depth, &min, &max);
  img_depth.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);
  cv::applyColorMap(adjMap, falseColorMap, cv::COLORMAP_AUTUMN);

  // Show images
	// cv::imshow("Current", img_rgb);
	// cv::imshow("depth", img_depth);
	// cv::imshow("diff", img_diff);
  // cv::imshow("diff masked", img_diff_masked);
  // cv::imshow("binary", img_binary);
  // cv::imshow("depth test", falseColorMap);
  // cv::waitKey(10);
 }


bool PoseEstimation::Estimate_pose(pose_estimation::pose_est_srv::Request   &req,
		                pose_estimation::pose_est_srv::Response  &res)
{
  geometry_msgs::PoseArray posearray;

	std::vector<cv::Mat> object_points = Detect(img_rgb, img_depth, img_binary);
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
	}
	res.rel_object_poses = posearray;
  
  res.rel_object_ids = object_ids;


  //std::string out_str = "Number of detected objects: " + std::to_string(object_points.size());
	//ROS_INFO_STREAM(out_str);
	return true;
}

bool PoseEstimation::callClassifier(std::vector<cv::RotatedRect> rot_rects, std::vector<std::string> &labels, std::vector<int> &mask)
{
  classifier_pkg::classify_detections::Request cds_req;
  classifier_pkg::classify_detections::Response cds_res;

  
  cds_req.header.stamp = ros::Time::now();
  for (cv::RotatedRect& r:rot_rects)
  {
    classifier_pkg::Classification_params param;
    param.width = r.size.width;
    param.height = r.size.height;
    cds_req.params.push_back(param);
  }

  if(classify_detections.call(cds_req, cds_res))
  {
    labels = cds_res.names;
    mask = cds_res.mask;

    //std::string n_classified_obj = "Number of classified objects: " + std::to_string(labels.size());
    //ROS_INFO_STREAM(n_classified_obj);
    
    return true;
  }
  return false;
}

std::vector<cv::Mat> PoseEstimation::Detect(cv::Mat &img_rgb, cv::Mat &img_depth, cv::Mat &img_binary)
{
 

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  findContours(img_binary, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  cv::Mat filledContours = detector::FillContours(img_binary, contours);
  // cv::imshow("filled", filledContours);
  // cv::waitKey(10);

  // Find contours again after filling
  std::vector<std::vector<cv::Point>> contours2;
  findContours(filledContours, contours2, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Find median depth in contours
  std::vector<float> depth = detector::DepthWithinPerimeter(contours2, img_depth);

  // Find rotated rects
  std::vector<cv::RotatedRect> rot_rects = detector::FindRotatedRects(contours2);

  // detector::ShowDetections(img_rgb, contours2,rot_rects);


  // Classify detections
  std::vector<std::string> labels;
  std::vector<int> mask;
  
  bool res = callClassifier(rot_rects, labels, mask);
  
  if (validate)
  {

    for(int i=0; i<rot_rects.size(); ++i)
    {
      cv::RotatedRect pred_rot_rect = rot_rects.at(i);
      size_t maxIOU_gtIDX;
      float temp_maxIOU=0;
      for(size_t j =0; j<gt_rot_rect.size();++j)
      {
        cv::RotatedRect gt_rot = gt_rot_rect.at(j);
        float iou = iouRotatedRects(pred_rot_rect, gt_rot);
        if (iou > temp_maxIOU)
        {
          maxIOU_gtIDX = int(j);
          temp_maxIOU = iou;
        }
      }
      if (gtLabel.at(maxIOU_gtIDX) == labels[i])
      {
        true_positive++;
        ROS_INFO("LABEL is the same as GT");
      }
      // SAVE BEST IOU as GT and Pred pair
      cv::RotatedRect gt_temp = gt_rot_rect.at(maxIOU_gtIDX);
      std::string gt_lab = "";
      std::string pred_lab="";
      gt_lab.append(gtLabel.at(maxIOU_gtIDX).begin()+4,gtLabel.at(maxIOU_gtIDX).end());
      pred_lab.append(labels[i].begin()+4,labels[i].end());
      // Labels, centerX, centerY, width, height angele Pairwise (GT, pred)
      poseDataFile << gt_lab+","+pred_lab<<"," << std::to_string(temp_maxIOU)<<","<<std::to_string(gt_temp.center.x)<<","<< std::to_string(pred_rot_rect.center.x)<<","<< std::to_string(gt_temp.center.y)<<","<< std::to_string(pred_rot_rect.center.y)<<","<< std::to_string(gt_temp.size.width)<<","<<std::to_string(pred_rot_rect.size.width)<<","<<std::to_string(gt_temp.size.height)<<","<<std::to_string(pred_rot_rect.size.height)<<","<< std::to_string(gt_temp.angle)<<","<<std::to_string(pred_rot_rect.angle)<<std::endl;            
    }
    float accuracy = true_positive/tot_gt;
    std::cout<<"current Accuracy:\t"<< accuracy<<"\t"<<true_positive <<std::endl;
  }

  // Push correctly classified objects to the output vector
  std::vector<std::string> new_objects;
  std::vector<cv::RotatedRect> new_rot_rects;

  for (int i = 0; i < mask.size(); i++)
  {
    if (mask[i] == true)
    {
      new_objects.push_back(labels[i]);
      new_rot_rects.push_back(rot_rects[i]);
    }
  }

  object_ids = new_objects;

  if (new_objects.size() > 0)
  {
    // Save detections to file
    detector::SaveToFile(img_rgb, img_depth, new_rot_rects);

    // Draw detections
    detector::ShowDetections(img_rgb, contours2, new_rot_rects);
  }

  return detector::convert_2_transforms(new_rot_rects, depth, img_rgb.size().width, img_rgb.size().height, f_x, f_y, camera2base);
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


void PoseEstimation::OnDynamicReconfigure(PoseEstimation::DynamicReconfigureType &config, uint32_t level)
{
  (void)level;
  config_ = config;
  // Load background image
  img_empty_background = cv::imread(config_.empty_img_path);
  //std::cout << "reconfig" << std::endl;

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
  camera_sync_->registerCallback(boost::bind(&PoseEstimation::OnImage, this, _1, _1));
  }

  // Service
  if(!service_started)
  {
    service_ = nh_.advertiseService(config_.output_topic, &PoseEstimation::Estimate_pose, this);
    ROS_INFO("Ready to estimate relative position of the object.");
    service_started = true;
  }
}

void PoseEstimation::compareToGT(pose_estimation::groundTruth msg)
{
  gtLabel.clear();
  gt_rot_rect.clear();
  for (int i=0; i<msg.cx.size(); i++)
  {
    tot_gt++;
    cv::Point2f center_p(msg.cx[i], msg.cy[i]); 
    cv::RotatedRect tmp_rect= cv::RotatedRect(center_p, cv::Size2f(msg.width[i], msg.height[i]), msg.angle[i]);
    gt_rot_rect.push_back(tmp_rect); 
    std::string temp_label = msg.label.at(i);
    gtLabel.push_back(temp_label);
  }
  loadNextImagePair(msg.rgbFile, msg.depthFile);

  return;
}

float PoseEstimation::iouRotatedRects(cv::RotatedRect pred_rot_rects,cv::RotatedRect gt_rot_rects)
{

      // compute intersection area
    std::vector<cv::Point2f> intersections_unsorted;
    std::vector<cv::Point2f> intersections;
    cv::rotatedRectangleIntersection(pred_rot_rects, gt_rot_rects, intersections_unsorted);
    
    if (intersections_unsorted.size() < 3) {
      return 0;
    }
    // need to sort the vertices CW or CCW
    cv::convexHull(intersections_unsorted, intersections);

    // Shoelace formula
    float intersection_area = 0;
    for (unsigned int i = 0; i < intersections.size(); ++i) {
      const auto& pt = intersections[i];
      const unsigned int i_next = (i + 1) == intersections.size() ? 0 : (i + 1);
      const auto& pt_next = intersections[i_next];
      intersection_area += (pt.x * pt_next.y - pt_next.x * pt.y);
    }
    intersection_area = std::abs(intersection_area) / 2;

    // compute union area
    const float area_GT = gt_rot_rects.size.area();
    const float area_detection = pred_rot_rects.size.area();
    const float union_area = area_GT + area_detection - intersection_area;
    
    // intersection over union
    const float overlap_score = intersection_area / union_area;
    return abs(overlap_score);
}


void PoseEstimation::loadNextImagePair(std::string rgbFile, std::string depthFile)
{
  img_rgb = cv::imread(rgbFile);
  img_depth = cv::imread(depthFile);

   // Get diff image
  bool use_DiffNorm = 1;
  if(use_DiffNorm)
  {
    img_diff = detector::DiffNorm(img_rgb, img_empty_background);
  }
  else
  {
    cv::absdiff(img_rgb, img_empty_background, img_diff);
    
    // Grey
    cv::cvtColor(img_diff, img_diff, cv::COLOR_BGR2GRAY);
  }

  // Apply mask
  img_diff_masked = detector::ApplyMask(img_diff, config_.mask_x, config_.mask_y, config_.mask_w, config_.mask_h);

  // Threshold back projected image to create a binary mask og the projected image
  cv::threshold(img_diff_masked, img_binary, config_.threshold_binary, 255.0, cv::THRESH_BINARY);

  // Erode binary image
  img_binary = detector::ErodeAndDilate(img_binary, config_.erosion_size, config_.dilation_size);

  // Image for testing depth registration
  double max = 1;
  double min = 0;
  cv::Mat falseColorMap;
  cv::Mat adjMap;
  cv::minMaxIdx(img_depth, &min, &max);
  img_depth.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);
  cv::applyColorMap(adjMap, falseColorMap, cv::COLORMAP_AUTUMN); 
}