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
    Pose next_empty; 
    int dim[2] = {5,5};
    bool is_kit_full=false;
    std::vector<Module> mount_matrix;
    bool grid_matrix[5][5];
    geometry_msgs::Pose origo_mount;
    geometry_msgs::Pose origo_grip_pose;
    TMatrix frame, grip_frame;


    void init_grid();

    void test();
    
public:
    AssemblyKit(/* args */);
    bool addModule(Object _obj);
    ~AssemblyKit();
};

#endif