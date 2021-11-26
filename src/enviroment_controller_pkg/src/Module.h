#ifndef MODULE_H
#define MODULE_H

#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/Pose.h"
#include "Matrix.h"
#include "Matrix.h"


struct Pose
{
    int x=1;
    int y=1;
};


class Module
{

protected:
    int module_id;
    float step_size = 0.050; //m

public:
    Module(/* args */);
    Module(int id, std::string type, Pose position, geometry_msgs::Pose pose_for_drop_off,geometry_msgs::Pose pose_for_approach);

    int getID();
    void setID(int);
    Pose pose_in_kit;
    bool object_in_module = false;
    std::string module_type;
    TMatrix approach_pose;
    TMatrix drop_off_pose;


    ~Module();
};



#endif
