#pragma once
#include <ros/ros.h>
#include <iostream>
#include "position_controller_pkg/Tcp_move.h"
#include <ur_robot_pkg/p2p_cmove.h>
#include <ur_robot_pkg/RobState.h>
#include <ur_robot_pkg/jointQs.h>
#include <ur_robot_pkg/CurrTCPPose.h>
//#include <ros/std_srvs/Trigger>
#include "position_controller_pkg/Pre_def_pose.h"
#include <geometry_msgs/Pose.h>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

enum Errors
{
    SERVICE_NOT_SUCC_COMPLETED = -6,
    NOT_CONNECTED = -5,
    EMERGENCY_STOP = -4,
    PROTECTIVE_STOP = -3,
    ROBOT_NOT_STEADY =-2,
    TIMEOUT = -1,
    SUCCESS = 1
};

class Position_control
{
private:

    /* data */
	ros::NodeHandle n;
    ros::ServiceServer pos_service;
    ros::ServiceServer move2DefPos_service;
    ros::ServiceClient p2p_service;
    ros::ServiceClient robState_service;
    int32_t max_waiting_time= -1; // mili seconds
    double std_vel = 1.25;
    double std_acc = 0.20;
    /* Functions */
    int move2Pose(position_controller_pkg::Tcp_move::Request &req, ur_robot_pkg::p2p_cmove::Response & res);
    int getRobState();
public:
    Position_control();
    bool position_controller(position_controller_pkg::Tcp_move::Request   &req, position_controller_pkg::Tcp_move::Response  &res);
    bool move2DefPos (position_controller_pkg::Pre_def_pose::Request &req, position_controller_pkg::Pre_def_pose::Response &res);

    ~Position_control();
};
