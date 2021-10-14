#ifndef MASTER_NODE_H
#define MASTER_NODE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include "pose_estimation/pose_est_srv.h"

#include "position_controller_pkg/Tcp_move.h"
#include "position_controller_pkg/Pre_def_pose.h"

#include "master_pkg/system_state_srv.h"
#include "master_pkg/gripper_Move.h"
#include "master_pkg/gripper_Conf.h"
 
#define home_pos_index  0
#define drop_off_pos_index 1

enum State
{
  error,
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
  int callServicePoseEstimate();
  bool callServiceTcpMove();
  bool callServiceGripperMove(float width,float speed);
  bool callServiceGripperGrasp(float width,float speed);
  bool callServiceGripperSetForce(float force);
  bool callServicePreMove(int pose_id);

  

  int setupServices();
  bool setupNodes();

  bool sendSystemState(master_pkg::system_state_srv::Request  &req,
                                master_pkg::system_state_srv::Response &res);

  ros::ServiceClient pose_estim_client;

  ros::ServiceClient gripper_move_client;
  ros::ServiceClient gripper_grasp_client;
  ros::ServiceClient gripper_set_force_client;

  ros::ServiceClient tcp_control_client;
  ros::ServiceClient tcp_pre_def_control_client;
  
  ros::ServiceServer system_state_server;

  
  State state = init;
  ros::NodeHandle n;
  const char *state_name[8] = { "error","init", "get_pose",
                            "move_to_pose","grasp_obj", "move_with_obj",
                            "drop_obj", "home"};

  geometry_msgs::Pose obj_pose;
};

#endif // MASTER_NODE_H