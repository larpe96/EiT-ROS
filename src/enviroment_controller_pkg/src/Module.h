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
    int id;
    float step_size = 0.050; //m

public:
    Module(/* args */);
    Module(std::string type, std::vector<Pose>);

    int getSize();
    void setID(int);
    int getID();
    std::vector<Pose> poses;
    bool object_in_module = false;
    std::string type;

    TMatrix dropOffPose;


    ~Module();
};



#endif
