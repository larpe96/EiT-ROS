#include "ros/ros.h"
#include "position_controller_pkg/Tcp_move.h"
#include "Position_control.hpp"

// bool position_controller(position_controller_pkg::Tcp_move::Request   &req, 
// 		    position_controller_pkg::Tcp_move::Response  &res)
// {
// 	// Only for viewving progess in the terminal
// 	ROS_INFO("position controller "); 
// 	return true; 
// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller_server");
	Position_control h;
	// ros::NodeHandle n; 
	
	// ros::ServiceServer service = n.advertiseService("position_controller", position_controller);
	ros::spin(); 
	
	return 0; 
}