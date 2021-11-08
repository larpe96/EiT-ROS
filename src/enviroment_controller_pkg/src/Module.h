#ifndef MODULE_H
#define MODULE_H

#include <iostream>
#include "Object.h"
#include <vector>
#include <string>
#include "geometry_msgs/Pose.h"
#include "Matrix.h"



struct Occu_pos
{
    int x=1;
    int y=1;
};

class Module
{

protected:
    Object object;
    bool objectInModule;
    float step_size = 0.050; //m
    int size; // # of holes the mount occupies
    TMatrix mountFrame, motherFrame, gripMother;
    std::vector<Occu_pos> occuPos;

public:
    Module(/* args */);
    Module(Object _obj, TMatrix _motherFrame, TMatrix _motherGripFrame);
    Module(Object _obj, TMatrix _motherFrame, Occu_pos _oc_pos);
    int getSize();
    std::vector<Occu_pos> getOccuPoses();
    void setOccuPoses(std::vector<Occu_pos> _vec);

    geometry_msgs::Pose getPlacePose();
    //geometry_msgs::Pose getApproachPlacePose();
    ~Module();
};



#endif
