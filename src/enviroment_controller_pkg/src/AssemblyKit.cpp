#include "AssemblyKit.h"

AssemblyKit::AssemblyKit(/* args */)
{
    // is_kit_full = false;
    // frame.eye();
    // std::cout << "here1" << std::endl;
    // init_grid(5,5);
    //jsonLoader::test();
    

    // // std::cout << "here2" << std::endl;
    // // std::vector<Pose> temp_vec;
    // // Pose temp_pose;
    // // temp_pose.x = 1; 
    // // temp_pose.y = 1;
    // // temp_vec.push_back(temp_pose); 
    // // temp_pose.y = 2;
    // // temp_vec.push_back(temp_pose);

    // // temp_pose.y = 3;
    // // temp_vec.push_back(temp_pose);

    // // addModule("typeA",temp_vec);
    // // std::cout << "here3" << std::endl;
    // // printGrid();
    // // temp_vec.clear();
    // // temp_pose;
    // // temp_pose.x = 2; 
    // // temp_pose.y = 1;
    // // temp_vec.push_back(temp_pose); 
    // // temp_pose.y = 2;
    // // temp_vec.push_back(temp_pose);

    // // temp_pose.y = 3;
    // // temp_vec.push_back(temp_pose);
    // // addModule("testB",temp_vec);
    // // modules[1].object_in_module = true;
    // // std::cout << "here3.5" << std::endl;
    // // printGrid();
    // // std::cout << numberOfModules()<< std::endl;
    // // removeModule(modules[0]);
    // // std::cout << numberOfModules()<< std::endl;
    // // printGrid();
    // origo_mount.position.x =  -0.255;
    // origo_mount.position.y = -0.81623;
    // origo_mount.position.z = -0.1;
    // // origo_mount.orientation.x = 0.1976573;
    // // origo_mount.orientation.y = 0.9802712;
    // // origo_mount.orientation.z = 0.0;
    // // origo_mount.orientation.w = 0.;
    
    // origo_mount.orientation.x = 0;
    // origo_mount.orientation.y = 0.;
    // origo_mount.orientation.z = 0.1976573;
    // origo_mount.orientation.w = 0.9802712; 

    // geometry_msgs::Pose pose_new;
    // pose_new.position.x =  0.05;
    // pose_new.position.y =  0.05;
    // pose_new.position.z = 0;
    // pose_new.orientation.x = 0.3826834;
    // pose_new.orientation.y = 0.9238795;
    // pose_new.orientation.z =  0;
    // pose_new.orientation.w =  0;

    // TMatrix base_to_assembly_origo(origo_mount);
    
    // std::cout << pose_new<< std::endl;

    // TMatrix test(pose_new);
    // TMatrix inv_test = test.inverse();

    // std::cout << inv_test.getPose()<< std::endl;

    // TMatrix test2(pose_new);

    // test2 = base_to_assembly_origo*test2;

    // test2.print();
    // std::cout << test2.getPose() <<std::endl;


}
AssemblyKit::AssemblyKit(std::vector<std::vector<int>>new_grid_matrix,geometry_msgs::Pose new_origo)
{
    grid_matrix = new_grid_matrix;
    TMatrix temp(new_origo);
    base_to_assembly_origo = temp;
}

AssemblyKit::~AssemblyKit()
{
}


/// PROTECTED

void AssemblyKit::init_grid(int size_y,int size_x)
{
    std::vector<std::vector<int>> grid;
    std::vector<int> temp_grid;
    for (int i = 0; i < size_x; ++i)
    {
        temp_grid.push_back(0);
    }
    for (int i = 0; i < size_y; ++i)
    {
        grid.push_back(temp_grid);
    }
    grid_matrix = grid;
}


int AssemblyKit::numberOfModules()
{
    return modules.size();
}


int AssemblyKit::findFreeModuleIndex(std::string type)
{
    int res = -1;
    if (kit_full == true)
    {
        return res;
    }
    for (int i = 0; i < modules.size(); i++)
    {
        std::cout <<modules[i].module_type << type << std::endl;
        if (modules[i].module_type == type)
        {
            if (modules[i].object_in_module == false)
            {
                return i;
            }
        }
    }
    return res;
}


void AssemblyKit::printGrid()
{
    std::cout << "Start print" <<std::endl;
    for (int x = 0; x < grid_matrix.size(); x++)
    {
        for (int y = 0; y < grid_matrix[0].size(); y++)
        {
            std::cout << grid_matrix[x][y];
        }
        std::cout << ""<< std::endl;
    }
    std::cout << ""<< std::endl;
}

void AssemblyKit::setModuleoccupied(int module_index)
{
    modules[module_index].object_in_module = true;

    for( int i = 0; i < modules.size();i++)
    {
        if (modules[module_index].object_in_module == false)
        {
            break;
        }
        if (i == modules.size())
        {
            kit_full = true;
        }
    }

}
geometry_msgs::Pose AssemblyKit::getModuleDropOffPose(int module_index)
{
    TMatrix assembly_to_module;
    Pose module_coordiante = modules[module_index].pose_in_kit;
    assembly_to_module.setTranslation(module_coordiante.x*0.05, module_coordiante.y*0.05, 0.0);

    TMatrix base_to_module = base_to_assembly_origo * assembly_to_module * modules[module_index].drop_off_pose;

    return  base_to_module.getPose();
}



geometry_msgs::Pose AssemblyKit::getModuleApproachPose(int module_index)
{
    TMatrix assembly_to_module;
    Pose module_coordiante = modules[module_index].pose_in_kit;
    assembly_to_module.setTranslation(module_coordiante.x*0.05, module_coordiante.y*0.05, 0.0);

    TMatrix base_to_module = base_to_assembly_origo * assembly_to_module * modules[module_index].approach_pose;

    return  base_to_module.getPose();
}
