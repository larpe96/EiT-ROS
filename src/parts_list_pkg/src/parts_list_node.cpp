#include <ros/ros.h>
#include <string>
#include "parts_list_pkg/partslist.h"
#include "parts_list_pkg/missing_parts.h"

PartsList bill;

bool MP_service_handler(parts_list_pkg::missing_parts::Request &req, parts_list_pkg::missing_parts::Response  &res)
{
    res.missing_parts = bill.checkMissingParts(req.detected_parts);
    return true; 
}

int main(int argc, char** argv)
{
  
  // Setup parts list object
  std::string partslist_file = "../workspace/src/parts_list_pkg/etc/parts_file.txt"; 
  bill.loadPartsList(partslist_file);
  ROS_INFO_STREAM("Number of objects in parts list: " + std::to_string(bill.getPartsList().size()));

  //Setup ros Node/service
  ros::init(argc, argv, "parts_list_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("missing_parts_srv", MP_service_handler);
  ROS_INFO("Ready to find missing parts");
  ros::spin();

}