#ifndef MASTER_NODE_H
#define MASTER_NODE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "pose_estimation/pose_est_srv.h"
#include "position_controller_pkg/Tcp_move.h"
#include "master_pkg/system_state_srv.h"
#include "master_pkg/gripper_Move.h"
 

enum State
{
   init,
   get_pose,
   move_to_pose,
   grasp_obj,
   move_with_obj,
   drop_obj,
   home
};



class MasterNode
{
public:
  //! Constructor.
  MasterNode();
  
  //! Destructor.
  ~MasterNode();

  void stateLoop(); 
protected:
  pose_estimation::pose_est_srv callServicePoseEstimate();
  position_controller_pkg::Tcp_move callServiceTcpMove();
  master_pkg::gripper_Move callServiceGripperMove();

  int setupServices();

  bool sendSystemState(master_pkg::system_state_srv::Request  &req,
                                master_pkg::system_state_srv::Response &res);

  ros::ServiceClient pose_estim_client;
  ros::ServiceClient gripper_control_client;
  ros::ServiceClient tcp_control_client;
  
  ros::ServiceServer system_state_server;

  
  State state = init;
  ros::NodeHandle n;
  const char *state_name[8] = { "init", "get_pose",
                            "move_to_pose","grasp_obj", "move_with_obj",
                            "drop_obj", "home"};
};

#endif // MASTER_NODE_H