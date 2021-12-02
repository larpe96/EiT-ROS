#pragma once

#include <iostream>
#include <fstream>
#include "pickup_db/pickup_db_srv.h"
#include <ros/ros.h>
#include <string>
#include <ros/service_server.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
namespace pt = boost::property_tree;




class PickupDB
{
  public:

    PickupDB();

    void Initialize(const ros::NodeHandle &nh);

    bool OnPose(pickup_db::pickup_db_srv::Request &req,
                pickup_db::pickup_db_srv::Response &res);

    geometry_msgs::Pose GetPickupPose(std::string object_type, geometry_msgs::Pose object_pose);
    geometry_msgs::Pose GetApproachPose(geometry_msgs::Pose pose_pickup);
    float GetOpenWidth(std::string object_type);

    void LoadJson(std::string path);

  private:
    /// Node handler
    ros::NodeHandle nh_;
    /// ROS service
    ros::ServiceServer service_;
    /// Root for object tree from json-file
    pt::ptree root;
    /// Object types subtree
    pt::ptree object_types;
};