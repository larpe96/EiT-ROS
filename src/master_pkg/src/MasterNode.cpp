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

int MasterNode::setupServices()
{
  int setupBool = 1;

  //Wait for the service
  setupBool = ros::service::waitForService("pose_est", 10);

  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("position_controller", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  setupBool = ros::service::waitForService("wsg_50_driver/move", 10);
  if (setupBool == 0)
  {
    return 0;
  }
  
  //client services
  pose_estim_client = n.serviceClient<pose_estimation::pose_est_srv>("pose_est");
  tcp_control_client = n.serviceClient<position_controller_pkg::Tcp_move>("position_controller");
  gripper_control_client = n.serviceClient<master_pkg::gripper_Move>("wsg_50_driver/move");

  //server services 
  system_state_server = n.advertiseService("system_state", &MasterNode::sendSystemState,this);
  return 1;
}

master_pkg::gripper_Move MasterNode::callServiceGripperMove()
{
    master_pkg::gripper_Move msg;
    msg.request.width = 50.0;
    msg.request.speed = 50.0;

    if(gripper_control_client.call(msg))
    {
        return msg;
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", gripper_control_client.getService().c_str());
        return msg;
    }
}


pose_estimation::pose_est_srv MasterNode::callServicePoseEstimate()
{
    pose_estimation::pose_est_srv msg;

    if(pose_estim_client.call(msg))
    {
        return msg;
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", pose_estim_client.getService().c_str());
        return msg;
    }
}
position_controller_pkg::Tcp_move MasterNode::callServiceTcpMove()
{
    position_controller_pkg::Tcp_move msg;

    if(tcp_control_client.call(msg))
    {
        return msg;
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", tcp_control_client.getService().c_str());
        return msg;
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
      state = get_pose;
    }
    break;
  case  get_pose:
    callServiceGripperMove();
    // code block
    break;
  case  move_to_pose:
    // code block
    break;
  case  grasp_obj:
    // code block
    break;
  case  move_with_obj:
    // code block
    break;
  case  drop_obj:
    // code block
    break;
  case  home:
    // code block
    break;
  default:
    // code block
    break;
  }
}