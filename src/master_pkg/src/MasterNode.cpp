#include "MasterNode.h"


/*--------------------------------------------------------------------
 * masterNode()
 * Constructor.
 *------------------------------------------------------------------*/

MasterNode::MasterNode()
{
    state = init;


} // end masterNode()

/*--------------------------------------------------------------------
 * ~masterNode()
 * Destructor.
 *------------------------------------------------------------------*/

MasterNode::~MasterNode()
{

} // end ~masterNode()

bool MasterNode::sendSystemState(master_pkg::system_state_srv::Request  &req,
                                master_pkg::system_state_srv::Response &res)
{
  res.state = state;
  res.str_state = state_name[state];
  return true;
}

bool MasterNode::initGraspSeq(std_srvs::Trigger::Request  &req,
                                std_srvs::Trigger::Response &res)
{
  if (state == ready)
  {
    bool status = callServicePoseEstimate();
    callServiceMissingParts();

    state = get_pose;
    res.success = 1;
    res.message = "init grasp seq";
    return true;

  }
  else
  {
    res.success = 0;
    res.message = "system is running";
    return true;
  }

}

bool MasterNode::setupNodes()
{

  if (callServiceGripperSetForce(40.0) == 0 )
  {
    return 0;
  }

  if (callServiceGripperMove(100, 250) == 0)
  {
    return 0;
  }


  return 1;
}
int MasterNode::setupServices()
{
  int setupBool = 1;


  setupBool = ros::service::waitForService("pose_est", 10);
  if (setupBool == 0)
  {
    return 0;
   }
  setupBool = ros::service::waitForService("get_module_drop_off_poses", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("move2_pos_srv", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("SET/robot_home", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("wsg_50_driver/move", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("wsg_50_driver/grasp", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("wsg_50_driver/set_force", 10);
  if (setupBool == 0)
  {
    return 0;
  }

  setupBool = ros::service::waitForService("pickup_db", 10);
  if (setupBool == 0)
  {
    return 0;
  }

  setupBool = ros::service::waitForService("missing_parts_srv", 10);
  if (setupBool == 0)
  {
    return 0;
  }

  //client services
  pose_estim_client = n.serviceClient<pose_estimation::pose_est_srv>("pose_est");
  missing_parts_client = n.serviceClient<parts_list_pkg::missing_parts>("missing_parts_srv");
  tcp_control_client = n.serviceClient<position_controller_pkg::Tcp_move>("move2_pos_srv");
  robot_home_control_client = n.serviceClient<std_srvs::Trigger>("SET/robot_home");

  gripper_move_client = n.serviceClient<master_pkg::gripper_Move>("wsg_50_driver/move");
  gripper_grasp_client = n.serviceClient<master_pkg::gripper_Move>("wsg_50_driver/grasp");
  gripper_set_force_client = n.serviceClient<master_pkg::gripper_Conf>("wsg_50_driver/set_force");

  drop_off_poses_client = n.serviceClient<enviroment_controller_pkg::module_poses_srv>("get_module_drop_off_poses");
  pickup_db_client = n.serviceClient<pickup_db::pickup_db_srv>("pickup_db");

  //server services
  system_state_server = n.advertiseService("system_state", &MasterNode::sendSystemState,this);
  init_grasp_seq_server = n.advertiseService("init_grasp_seq", &MasterNode::initGraspSeq,this);
  return 1;
}

bool MasterNode::callServiceObjDropOff(std::string obj_type1)
{
    enviroment_controller_pkg::module_poses_srv srv;
    srv.request.obj_type = obj_type1;

    if(!drop_off_poses_client.call(srv))
    {
        ROS_ERROR("Failed to call service: %s", drop_off_poses_client.getService().c_str());
        return 0;
    }

    if(srv.response.success != 1)
    {
     ROS_ERROR("Generating an drop off pose failed.");
     return 0;
    }
    else
    {
      drop_off_pose = srv.response.drop_off_pose;
      approach_drop_off_pose = srv.response.approach_pose;
    }
   return 1;
}

bool MasterNode::callServiceGripperMove(float width,float speed)
{
    master_pkg::gripper_Move msg;
    msg.request.width = width;
    msg.request.speed = speed;

    if(!gripper_move_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", gripper_move_client.getService().c_str());
        return 0;
    }

    if(msg.response.error != 0)
    {
     ROS_ERROR("Gripper move failed. Error code: %d",msg.response.error);
     return 0;
    }
    else
    {
      return 1;
    }
}
bool MasterNode::callServiceGripperGrasp(float width,float speed)
{
    master_pkg::gripper_Move msg;
    msg.request.width = width;
    msg.request.speed = speed;

    if(!gripper_grasp_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", gripper_grasp_client.getService().c_str());
        return 0;
    }

    if(msg.response.error != 0)
    {
     ROS_ERROR("Gripper grasp failed. Error code: %d",msg.response.error);
     return 0;
    }
   return 1;
}
bool MasterNode::callServiceGripperSetForce(float force)
{
    master_pkg::gripper_Conf msg;
    msg.request.val = force;

    if(!gripper_set_force_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", gripper_set_force_client.getService().c_str());
        return 0;
    }

    if(msg.response.error != 0)
    {
     ROS_ERROR("Gripper set force failed. Error code: %d",msg.response.error);
     return 0;
    }
    return 1;
}


void MasterNode::callServiceMissingParts()
{
    parts_list_pkg::missing_parts MP_msg;
    MP_msg.request.detected_parts = obj_ids;
    missing_parts_client.call(MP_msg);
    if(MP_msg.response.missing_parts.size() > 0)
    {
        std::string output = "Missing Parts: ";
        for(int i = 0; i < MP_msg.response.missing_parts.size(); i++)
        {
          output = output + MP_msg.response.missing_parts[i] + ",  ";
        }
        ROS_INFO_STREAM(output);
    }
    else
    {
        ROS_INFO_STREAM("Missing Parts: None");
    }
}


int MasterNode::callServicePoseEstimate()
{
    pose_estimation::pose_est_srv msg;

    if(pose_estim_client.call(msg))
    {
        if (msg.response.rel_object_poses.poses.size() == 0 or msg.response.rel_object_ids.size() == 0 )
          {
            return 2;
          }
        else
        {
          obj_pose = msg.response.rel_object_poses.poses[0];

          obj_ids = msg.response.rel_object_ids;

          return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", pose_estim_client.getService().c_str());
        return 0;
    }

}
bool MasterNode::callServiceTcpMove(geometry_msgs::Pose TCP_pose)
{
    position_controller_pkg::Tcp_move msg;

    msg.request.pose = TCP_pose;

    if(!tcp_control_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", tcp_control_client.getService().c_str());
        return 0;
    }

    if (msg.response.succes != 1)
    {

      ROS_ERROR("Robot could not move. Error code: %ld",msg.response.succes);
      return 0;
    }
    else
    {
      return 1;
    }
}

bool MasterNode::callServiceGoHome()
{
    std_srvs::Trigger msg;
    if (!robot_home_control_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", robot_home_control_client.getService().c_str());
        return 0;
    }

    if (msg.response.success != 1)
    {

      ROS_ERROR("Robot could not move. Error code: %d",msg.response.success);
      return 0;
    }
    else
    {
      return 1;
    }
}

bool MasterNode::callServicePickupDB(std::string obj_type, geometry_msgs::Pose pose_estimated)
{
    pickup_db::pickup_db_srv msg;

    msg.request.object_type = obj_type;
    msg.request.object_pose = pose_estimated;

    if (!pickup_db_client.call(msg))
    {
      ROS_ERROR("Failed to call service: %s", pickup_db_client.getService().c_str());
      return 0;
    }
    else
    {
      pose_pickup = msg.response.pose_pickup;
      pose_pickup_approach = msg.response.pose_approach;
      gripper_open_width = msg.response.open_width;
      return 1;
    }
}

std::string MasterNode::getState()
{
  return state_name[state];
}

void MasterNode::stateLoop()
{
  switch(state) {
  case  init:

    if (setupServices())
    {

      if(setupNodes() == 0)
      {
        state = error;
      }
      else
      {
        state = callServiceGoHome() ? ready : error;
      }

    }
    break;
  case ready:
    {
      break;
    }
  case  get_pose:
    {
      int res = 0;
      obj_ids.clear();

      res = callServicePoseEstimate();
      
      if( res == 1 )
      {
        res = callServicePickupDB(obj_ids[0], obj_pose);
        if(res == 0)
        {
          state = error;
        }
        state = approach_pose;
      }
      else if(res == 0)
      {
        state = error;
      }
      else
      {
        state = ready;
      }
      break;
    }
  case approach_pose:
    {
      
      int res = 0;
      res = callServiceTcpMove(pose_pickup_approach);

      if (res == 1)
      {
        state = move_to_pose;
      }
      else
      {
        state = error;
      }
      break;
    }

  case  move_to_pose:
  {
    
    state = callServiceTcpMove(pose_pickup) ? grasp_obj : error;
    break;
    }
  case  grasp_obj:
    {
      state = error;
      if (callServiceGripperGrasp(gripper_open_width, 250) )
      {
        state = deproach_pose;
      }
      else 
      {
        if (callServiceGripperMove(100,250))
        {
          state = callServiceGoHome() ? get_pose : error;
        }
      }
      break;
    }
  case deproach_pose:
    {
      int res = 0;
      res = callServiceTcpMove(pose_pickup_approach);
      if (res == 1)
      {
        state = callServiceGoHome() ? move_to_approach_drop_off : error;
        //state = move_to_approach_drop_off;
      }
      else
      {
        state = error;
      }
      break;
    }
  case  move_to_approach_drop_off:
    {
      int res = callServiceObjDropOff(obj_ids[0]);
      if (res == 0)
      {
        state = error;
      }
      else
      {
        state = callServiceTcpMove(approach_drop_off_pose) ? move_to_drop_off : error;
      }
      break;
    }
  case  move_to_drop_off:
    state = callServiceTcpMove(drop_off_pose) ? drop_obj : error;
    break;
  case  drop_obj:
    state = callServiceGripperMove(100,250) ? move_to_deproach_drop_off : error;
    break;
  case  move_to_deproach_drop_off:
    state = callServiceTcpMove(approach_drop_off_pose) ? home : error;
    break;
  case  home:
    state = callServiceGoHome() ? get_pose : error;
    break;
  default:
    ROS_ERROR("Master State is in error state");
    break;
  }
}
