#include "Module.h"


Module::Module(/* args */)
{
}

Module::Module(int id, std::string type, Pose position, geometry_msgs::Pose pose_for_drop_off,geometry_msgs::Pose pose_for_approach)
{
   object_in_module = false;
   module_type = type;
   pose_in_kit = position;
  
   TMatrix temp(pose_for_drop_off);
   drop_off_pose = temp;

   TMatrix temp2(pose_for_approach);
   approach_pose = temp2;

   module_id = id;
}

Module::~Module()
{
}

int Module::getID()
{
   return module_id;
}

void Module::setID(int _id)
{
   module_id =  _id;
}