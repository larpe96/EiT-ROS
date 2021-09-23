#include "ros/ros.h"
#include "position_controller_pkg/Tcp_move.h"

bool position_controller(position_controller_pkg::Tcp_move::Request   &req, 
		    position_controller_pkg::Tcp_move::Response  &res)
{
	// Only for viewving progess in the terminal
	ROS_INFO("position controller "); 
	return true; 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller_server");
	ros::NodeHandle n; 
	
	ros::ServiceServer service = n.advertiseService("position_controller", position_controller);
	ROS_INFO("Ready to control the position.");
	ros::spin(); 
	
	return 0; 
}