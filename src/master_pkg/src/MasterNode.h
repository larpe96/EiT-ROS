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
#include "std_srvs/Trigger.h"

#include "pose_estimation/pose_est_srv.h"
#include "parts_list_pkg/missing_parts.h"

#include "position_controller_pkg/Tcp_move.h"
#include "position_controller_pkg/Pre_def_pose.h"

#include "enviroment_controller_pkg/module_poses_srv.h"

#include "master_pkg/system_state_srv.h"
#include "master_pkg/gripper_Move.h"
#include "master_pkg/gripper_Conf.h"

#define home_pose_name  "pose_2"
#define grasp_pose_name "pose_0"
#define approach_pose_name "pose_1"
#define drop_off_pose_name "pose_99"

enum State
{
  error,
  init,
  ready,
  get_pose,
  approach_pose,
  move_to_pose,
  grasp_obj,
  deproach_pose,
  move_to_approach_drop_off,
  move_to_drop_off,
  drop_obj,
  move_to_deproach_drop_off,
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
  std::string getState();

protected:
  int callServicePoseEstimate();
  void callServiceMissingParts();
  bool callServiceTcpMove(geometry_msgs::Pose);
  bool callServiceGripperMove(float width,float speed);
  bool callServiceGripperGrasp(float width,float speed);
  bool callServiceGripperSetForce(float force);
  bool callServicePreMove(std::string pose_name);
  bool callServiceObjDropOff(std::string obj_type);



  int setupServices();
  bool setupNodes();

  bool sendSystemState(master_pkg::system_state_srv::Request  &req,
                                master_pkg::system_state_srv::Response &res);
  
  bool initGraspSeq(std_srvs::Trigger::Request  &req,
                                std_srvs::Trigger::Response &res);

  ros::ServiceClient pose_estim_client;
  ros::ServiceClient missing_parts_client;

  ros::ServiceClient gripper_move_client;
  ros::ServiceClient gripper_grasp_client;
  ros::ServiceClient gripper_set_force_client;

  ros::ServiceClient tcp_control_client;
  ros::ServiceClient tcp_pre_def_control_client;

  ros::ServiceClient drop_off_poses_client;

  ros::ServiceServer system_state_server;
  ros::ServiceServer init_grasp_seq_server;
  


  State state = init;
  ros::NodeHandle n;
  const char *state_name[13] = { "error","init", "ready", "get_pose","approach_pose",
                            "move_to_pose","grasp_obj", "deproach_pose" , "move_to_approach_drop_off",
                            "move_to_drop_off","drop_obj","move_to_deproach_drop_off", "home"};

  geometry_msgs::Pose obj_pose;
  std::vector<std::string> obj_ids;
  
  geometry_msgs::Pose approach_drop_off_pose;
  geometry_msgs::Pose drop_off_pose;
};

#endif // MASTER_NODE_H
