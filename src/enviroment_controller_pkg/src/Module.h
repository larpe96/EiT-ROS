#ifndef MODULE_H
#define MODULE_H

#include <iostream>
#include "Object.h"
#include <vector>
#include <string>
#include "geometry_msgs/Pose.h"
#include "Matrix.h"



struct Pose
{
    int x=1;
    int y=1;
};

class Module
{

protected:
    Object object;
    bool object_in_module;
    float step_size = 0.050; //m
    int size; // # of holes the mount occupies
    std::vector<Pose> poses;

public:
    Module(/* args */);
    Module(Object _obj, TMatrix _motherFrame, TMatrix _motherGripFrame);
    Module(Object _obj, TMatrix _motherFrame, Pose _oc_pos);
    int getSize();
    std::vector<Pose> getOccupiedPoses();
    void setOccupiedPoses(std::vector<Pose> _vec);

    geometry_msgs::Pose getPlacePose(); //slet
    //geometry_msgs::Pose getApproachPlacePose();
    ~Module();
};



#endif
