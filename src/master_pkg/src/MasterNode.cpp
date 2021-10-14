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

bool MasterNode::setupNodes()
{
  
  if (callServiceGripperSetForce(40.0) == 0 )
  {
    return 0;
  }

  if (callServiceGripperMove(50, 100) == 0)
  {
    return 0;
  }
  return 1;
}
int MasterNode::setupServices()
{
  int setupBool = 1;

  //Wait for the service
  // setupBool = ros::service::waitForService("pose_est", 10);

  // if (setupBool == 0)
  // {
  //   return 0;
  // }
  setupBool = ros::service::waitForService("move2_pos_srv", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("move2_def_pos_srv", 10);
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
  
  
  //client services
  // pose_estim_client = n.serviceClient<pose_estimation::pose_est_srv>("pose_est");
  tcp_control_client = n.serviceClient<position_controller_pkg::Tcp_move>("move2_pos_srv");
  tcp_pre_def_control_client = n.serviceClient<position_controller_pkg::Pre_def_pose>("move2_def_pos_srv");

  gripper_move_client = n.serviceClient<master_pkg::gripper_Move>("wsg_50_driver/move");
  gripper_grasp_client = n.serviceClient<master_pkg::gripper_Move>("wsg_50_driver/grasp");
  gripper_set_force_client = n.serviceClient<master_pkg::gripper_Conf>("wsg_50_driver/set_force");

  //server services 
  system_state_server = n.advertiseService("system_state", &MasterNode::sendSystemState,this);
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


int MasterNode::callServicePoseEstimate()
{
    pose_estimation::pose_est_srv msg;

    if(pose_estim_client.call(msg))
    {
        if (msg.response.rel_object_poses.poses.size() == 0)
          {
            return 2;
          }
        else 
        {
          obj_pose = msg.response.rel_object_poses.poses[0];
          return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", pose_estim_client.getService().c_str());
        return 0;
    }

}
bool MasterNode::callServiceTcpMove()
{
    position_controller_pkg::Tcp_move msg;

    msg.request.pose = obj_pose;

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

bool MasterNode::callServicePreMove(int pose_id)
{
    position_controller_pkg::Pre_def_pose msg;

    msg.request.pre_def_pose = pose_id;

    if (!tcp_pre_def_control_client.call(msg))
    {
        ROS_ERROR("Failed to call service: %s", tcp_pre_def_control_client.getService().c_str());
        return 0;
    }

    if (msg.response.succes != 1)
    {
      
      ROS_ERROR("Robot could not move. Error code: %d",msg.response.succes); 
      return 0;
    }
    else
    {
      return 1;
    }
}

void MasterNode::stateLoop()
{

  //pose_estimation::pose_est_srv msg = callServicePoseEstimate();

  //std::cout << msg.response << std::endl;
  std::cout << state_name[state] << std::endl;

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
        state = callServicePreMove(home_pos_index) ? get_pose : error;
      }
      
    }
    break;
  case  get_pose:
    {
      int res = 1;// callServicePoseEstimate(); 
      if( res == 1 )
      {
        state = move_to_pose;
      }
      else if(res == 0)
      {
        state = error;
      }
      else
      {
        state = get_pose;
      }
      break;
    }
  case  move_to_pose:
    //state = callServiceTcpMove() ? grasp_obj : error;
    state = callServicePreMove(2) ? grasp_obj : error;
    break;
  case  grasp_obj:
    state = callServiceGripperGrasp(22,50) ? move_with_obj : error;
    break;
  case  move_with_obj:
    state = callServicePreMove(drop_off_pos_index) ? drop_obj : error;
    break;
  case  drop_obj:
    state = callServiceGripperMove(50,100) ? home : error;
    break;
  case  home:
    state = callServicePreMove(home_pos_index) ? get_pose : error;
    break;
  default:
    ROS_ERROR("Master State is in error state");
    break;
  }
}
