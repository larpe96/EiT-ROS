#include "AssemblyKit.h"

AssemblyKit::AssemblyKit(/* args */)
{
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
