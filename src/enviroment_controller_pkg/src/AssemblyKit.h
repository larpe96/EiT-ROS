#ifndef ASSEMBLYKIT_H
#define ASSEMBLYKIT_H

#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include<string>
#include <vector>
#include "Module.h"
#include "Matrix.h"


class AssemblyKit
{
protected:
    // int id;
    // int status;
    // std::string project_name;
    // std::string manifold_type;
    Occu_pos next_empty; 
    int dim[2] = {5,5};
    bool isKitFull=false;
    std::vector<Module> mountMatrix;
    bool occuGrid[5][5];
    geometry_msgs::Pose origoMount;
    geometry_msgs::Pose origoGripPose;
    TMatrix frame, gripFrame;


    void init_occuGrid();
    void increase_nect_empt();
    bool check_mount_grid(Module* _m);
    
    void setOrientation();
    void test();

    
public:
    AssemblyKit(/* args */);
    void setOrigoMount(geometry_msgs::Point _origoMount);
    bool add_module(Object _obj);



    ~AssemblyKit();
};

#endif