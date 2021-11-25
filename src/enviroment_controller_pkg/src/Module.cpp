#include "Module.h"


Module::Module(/* args */)
{
}

Module::Module(std::string _module_type, std::vector<Pose> _occupied_poses)
{
    object_in_module = false;
    type = _module_type;
    poses = _occupied_poses;
}

Module::~Module()
{
}

int Module::getSize()
{
    return poses.size();
}

void Module::setID(int _id)
{
    id = _id;
}

int Module::getID()
{
   return id;
}