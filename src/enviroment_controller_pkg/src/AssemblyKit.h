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
    int id;

    int dim[2] = {5,5};
    bool is_kit_full=false;
    std::vector<Module> modules;
    std::vector<std::vector<int>> grid_matrix;
    TMatrix frame;  
    geometry_msgs::Pose origo_mount;
    
    //origo_mount.position.x =  -0.255;
    // origo_mount.position.y = -0.81623;
    // origo_mount.position.z = -0.1;
    // origo_mount.orientation.x = 0.15;
    // origo_mount.orientation.y = -1.;
    // origo_mount.orientation.z = 0;
    // origo_mount.orientation.w = 0;
    TMatrix base_to_assembly_origo;
    
    int id_counter = 0;

    void init_grid(int,int);
    
    
public:
    AssemblyKit(/* args */);
    bool addModule(std::string _type,std::vector<Pose> _module_poses);
    bool removeModule(Module _module);
    int numberOfModules();
    int findFreeModuleIndex(std::string type);
    geometry_msgs::Pose getModulePose(int moduleIndex);



    void printGrid();
    void updateGridMatrix();
    ~AssemblyKit();
};

#endif