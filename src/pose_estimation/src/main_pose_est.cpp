#include <iostream>
#include "ros/ros.h"
#include "pose_estimation/pose_est_srv.h"

bool estimate_pose(pose_estimation::pose_est_srv::Request   &req, 
		    pose_estimation::pose_est_srv::Response  &res)
{
	// Only for viewving progess in the terminal
	res.rel_object_pose.position.x = 10;
	res.rel_object_pose.position.y = 10;
	res.rel_object_pose.position.z = 10;
	ROS_INFO("pose was estimated"); 
	return true; 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_est_server");
	ros::NodeHandle n; 
	
	ros::ServiceServer service = n.advertiseService("pose_est", estimate_pose);
	ROS_INFO("Ready to estimate relative position of the object.");
	ros::spin(); 
	
	return 0; 
}
