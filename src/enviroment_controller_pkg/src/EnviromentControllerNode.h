#ifndef ENVIROMENT_CONTROLLER_H
#define ENVIROMENT_CONTROLLER_H

#include <iostream>
#include <vector>
#include <string>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include"AssemblyKit.h"
#include <geometry_msgs/Point.h>
#include "std_srvs/Trigger.h"
#include "enviroment_controller_pkg/module_poses_srv.h"
#include "enviroment_controller_pkg/kit_info_srv.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <json.hpp>
using namespace std;
using json = nlohmann::json;
namespace pt = boost::property_tree;

#define service_list = 

class EnviromentControllerNode
{
public:
  //! Constructor.
  EnviromentControllerNode();

  //! Destructor.
  ~EnviromentControllerNode();
  void resetAssemblyKit();
protected:
  AssemblyKit loadAssemblyKit(std::string jsonpath,std::string filename);
  void loadModules(std::string jsonpath,std::string filename);
  Module loadModule(Pose module_position,int module_number ,std::string jsonpath,std::string filename);
  std::vector<std::vector<int>>getGridmatrix(std::string grid);
  std::string jsonpath = "/home/user/workspace/src/enviroment_controller_pkg/etc/";
  bool resetKit(std_srvs::Trigger::Request  &req,
                                std_srvs::Trigger::Response &res);
  bool sendModulePoses(enviroment_controller_pkg::module_poses_srv::Request  &req,
                                enviroment_controller_pkg::module_poses_srv::Response &res);
  bool getKitInfo(enviroment_controller_pkg::kit_info_srv::Request  &req,
                                enviroment_controller_pkg::kit_info_srv::Response &res);
  AssemblyKit assembly_kit;
  ros::NodeHandle n;

  ros::ServiceServer reset_kit_server;
  ros::ServiceServer get_module_poses_server;
  ros::ServiceServer get_kit_info_server;




};

#endif // ENVIROMENT_CONTROLLER_H
