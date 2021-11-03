#ifndef ENVIROMENT_CONTROLLER_H
#define ENVIROMENT_CONTROLLER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "pose_estimation/pose_est_srv.h"

#include "position_controller_pkg/Tcp_move.h"
#include "position_controller_pkg/Pre_def_pose.h"

class EnviromentControllerNode
{
public:
  //! Constructor.
  EnviromentControllerNode();

  //! Destructor.
  ~EnviromentControllerNode();

protected:


};

#endif // ENVIROMENT_CONTROLLER_H
