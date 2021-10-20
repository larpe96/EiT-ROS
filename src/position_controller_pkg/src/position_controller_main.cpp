#include "ros/ros.h"
#include "position_controller_pkg/Tcp_move.h"
#include "Position_control.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller_server");
	Position_control h;
	ros::spin(); 
	
	return 0; 
}