#include "Module.h"


Module::Module(/* args */)
{
}

Module::Module(Object _obj, TMatrix _motherFrame, TMatrix _motherGripFrame )
{
    object = _obj;
    objectInModule = false;
    size = 1;
    motherFrame = _motherFrame;
    gripMother = _motherGripFrame;
}

Module::Module(Object _obj, TMatrix _motherFrame, Occu_pos _oc_pos)
{
    object = _obj;
    objectInModule = false;
    size = 1;
    occuPos.push_back(_oc_pos);
    mountFrame.setTranslation(_oc_pos.x*step_size, _oc_pos.y*step_size, 0.10);
}

Module::~Module()
{
}

int Module::getSize()
{
    return size;
}

void Module::setOccuPoses(std::vector<Occu_pos> _vec)
{
    occuPos = _vec;
    Occu_pos _oc_pos = occuPos.at(0);
    mountFrame.setTranslation(_oc_pos.x*step_size, _oc_pos.y*step_size, 0.10);
}

std::vector<Occu_pos> Module::getOccuPoses()
{
    return occuPos;
}

geometry_msgs::Pose Module::getPlacePose()
{
    TMatrix rel2World = (motherFrame*mountFrame*gripMother).inverse();
    std::cout<<"rel2World\n";
    rel2World.print();
    return rel2World.getPose();
}