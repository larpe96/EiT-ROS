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
    bool kit_full=false;
    
    TMatrix frame;  
    geometry_msgs::Pose origo_mount;

    TMatrix base_to_assembly_origo;
    
    int id_counter = 0;

    void init_grid(int,int);
    
    
public:
    AssemblyKit(/* args */);
    AssemblyKit(std::vector<std::vector<int>>new_grid_matrix,geometry_msgs::Pose new_origo);
    int numberOfModules();
    int findFreeModuleIndex(std::string type);
    geometry_msgs::Pose getModulePose(int moduleIndex);
    std::vector<std::vector<int>> grid_matrix;
    std::vector<Module> modules;
    void setModuleoccupied(int module_index);
    geometry_msgs::Pose getModuleDropOffPose(int module_index);
    geometry_msgs::Pose getModuleApproachPose(int module_index);


    void printGrid();
    void updateGridMatrix();
    ~AssemblyKit();
};

#endif