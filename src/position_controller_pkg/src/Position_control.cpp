#include "Position_control.hpp"


Position_control::Position_control()
{
    bool is_not_time_out = false;
    n = ros::NodeHandle();
    is_not_time_out = ros::service::waitForService("/SET/p2p_Cmove_srv",max_waiting_time);

    if(!is_not_time_out)
    {
        std::cerr<<"TIMEOUT!!"<<std::endl;
    }

    p2p_service = n.serviceClient<ur_robot_pkg::p2p_cmove>("/SET/p2p_Cmove_srv");

    is_not_time_out = ros::service::waitForService("GET/robot_state_srv", max_waiting_time);
    if(!is_not_time_out)
    {
        std::cerr<<"TIMEOUT!!"<<std::endl;
    }

    robState_service = n.serviceClient<ur_robot_pkg::RobState>("GET/robot_state_srv");
    move2DefPos_service = n.advertiseService("move2_def_pos_srv", &Position_control::move2DefPos, this);
    pos_service = n.advertiseService("move2_pos_srv", &Position_control::position_controller, this);

}

bool Position_control::position_controller(position_controller_pkg::Tcp_move::Request &req, position_controller_pkg::Tcp_move::Response  &res)
{
	// Only for viewving progess in the terminal
  // Only for viewving progess in the terminal
  ur_robot_pkg::p2p_cmove::Response response;
  int result = move2Pose(req,response);
  res.succes = result; // We did not return/update/set a response :o
  if (res.succes== SUCCESS)
  {
    return true;
  }
  else{
    return false;
  }
}

bool Position_control::move2DefPos(position_controller_pkg::Pre_def_pose::Request &req, position_controller_pkg::Pre_def_pose::Response &res)
{
    std::string pose_name = req.pre_def_pose;
    int rob_status = getRobState();
    if(rob_status != SUCCESS)
    {
        res.succes = SERVICE_NOT_SUCC_COMPLETED;
        return false;
    }

    std::fstream file;
    file.open("/home/user/workspace/src/position_controller_pkg/etc/"+pose_name+".eit");
    std::string str;
    if(!file.is_open())
    {
       std::cout<<"COULD NOT FIND FILE"<<std::endl;
       res.succes = SERVICE_NOT_SUCC_COMPLETED;
       return false;
    }

    std::vector<double> pos_elements;

    while (std::getline(file,str))
    {
       pos_elements.push_back(std::stod(str));
    }
    ur_robot_pkg::p2p_cmove srv;
    srv.request.acc = std_acc;
    srv.request.vel = std_vel;
    srv.request.move_async = false;
    geometry_msgs::Pose pose;
    pose.position.x = pos_elements.at(0);
    pose.position.y = pos_elements.at(1);
    pose.position.z = pos_elements.at(2);
    pose.orientation.x = pos_elements.at(3);
    pose.orientation.y = pos_elements.at(4);
    pose.orientation.z = pos_elements.at(5);
    pose.orientation.w = pos_elements.at(6);
    srv.request.pose = pose;
    bool rob_suc = p2p_service.call(srv);

    if (srv.response.succes)
    {
        res.succes = SUCCESS;
        return true;
    }
    res.succes = SERVICE_NOT_SUCC_COMPLETED;
    return false;

}


Position_control::~Position_control()
{
}


// --- Private --- //

int Position_control::move2Pose(position_controller_pkg::Tcp_move::Request &req, ur_robot_pkg::p2p_cmove::Response & res)
{
    int rob_status = getRobState();
    if( rob_status != SUCCESS)
    {
        return rob_status;
    }

    bool suc_rob_move = false, suc_resp=false;
    geometry_msgs::Pose pose_msgs = req.pose;
    ur_robot_pkg::p2p_cmove srv;
    srv.request.vel = std_vel;
    srv.request.acc = std_acc;
    srv.request.pose = pose_msgs;
    srv.request.move_async = false;
    suc_rob_move = p2p_service.call(srv);
    if (!suc_rob_move)
        return USUCCESSFUL_SERVICE_CALL;

    if (!srv.response.succes)
    {
      return SERVICE_UNSUCCSESSFUL;
    }

    res = srv.response;
    suc_resp = res.succes;
    if (suc_resp)
        return SUCCESS;
    return SERVICE_NOT_SUCC_COMPLETED;
}


int Position_control::getRobState()
{
    bool  is_conn, is_protec, is_Em_stop, is_steady;
    bool suc_robStat_call=false;
    ur_robot_pkg::RobState rob_state_srv;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point end;
    while (!suc_robStat_call && std::chrono::duration_cast<std::chrono::seconds>(end-start).count() < max_waiting_time)
    {
        suc_robStat_call = robState_service.call(rob_state_srv);
        end = std::chrono::high_resolution_clock::now();
    }

    if (suc_robStat_call == false)
        return TIMEOUT;

    ur_robot_pkg::RobState::Response rob_State = rob_state_srv.response;
    is_conn = rob_State.isConnected;
    is_protec = rob_State.isProtectiveStopped;
    is_Em_stop = rob_State.isEmergencyStopped;
    is_steady = rob_State.isSteady;

    if (!is_conn)
        return NOT_CONNECTED;
    if (is_protec)
        return PROTECTIVE_STOP;
    if (is_Em_stop)
        return EMERGENCY_STOP;
    if (! is_steady)
        return ROBOT_NOT_STEADY;
    return SUCCESS;
}
