#include "Module.h"


Module::Module(/* args */)
{
}

Module::Module(Object _obj, TMatrix _motherFrame, TMatrix _motherGripFrame )
{
    object = _obj;
    object_in_module = false;
    size = 1;
}

Module::Module(Object _obj, TMatrix _motherFrame, Pose _oc_pos)
{
    object = _obj;
    object_in_module = false;
    size = 1;
    poses.push_back(_oc_pos);
}

Module::~Module()
{
}

int Module::getSize()
{
    return poses.size();
}

void Module::setOccupiedPoses(std::vector<Pose> _vec)
{
    poses = _vec;
    Pose _oc_pos = poses.at(0);
}

std::vector<Pose> Module::getOccupiedPoses()
{
    return poses;
}