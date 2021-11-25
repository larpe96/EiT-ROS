#include "AssemblyKit.h"

AssemblyKit::AssemblyKit(/* args */)
{
    is_kit_full = false;
    frame.eye();
    std::cout << "here1" << std::endl;
    init_grid(5,5);
    // std::cout << "here2" << std::endl;
    // std::vector<Pose> temp_vec;
    // Pose temp_pose;
    // temp_pose.x = 1; 
    // temp_pose.y = 1;
    // temp_vec.push_back(temp_pose); 
    // temp_pose.y = 2;
    // temp_vec.push_back(temp_pose);

    // temp_pose.y = 3;
    // temp_vec.push_back(temp_pose);

    // addModule("typeA",temp_vec);
    // std::cout << "here3" << std::endl;
    // printGrid();
    // temp_vec.clear();
    // temp_pose;
    // temp_pose.x = 2; 
    // temp_pose.y = 1;
    // temp_vec.push_back(temp_pose); 
    // temp_pose.y = 2;
    // temp_vec.push_back(temp_pose);

    // temp_pose.y = 3;
    // temp_vec.push_back(temp_pose);
    // addModule("testB",temp_vec);
    // modules[1].object_in_module = true;
    // std::cout << "here3.5" << std::endl;
    // printGrid();
    // std::cout << numberOfModules()<< std::endl;
    // removeModule(modules[0]);
    // std::cout << numberOfModules()<< std::endl;
    // printGrid();
    origo_mount.position.x =  -0.255;
    origo_mount.position.y = -0.81623;
    origo_mount.position.z = -0.1;
    // origo_mount.orientation.x = 0.1976573;
    // origo_mount.orientation.y = 0.9802712;
    // origo_mount.orientation.z = 0.0;
    // origo_mount.orientation.w = 0.;
    
    origo_mount.orientation.x = 0;
    origo_mount.orientation.y = 0.;
    origo_mount.orientation.z = 0.1976573;
    origo_mount.orientation.w = 0.9802712; 

    geometry_msgs::Pose pose_new;
    pose_new.position.x =  0.05;
    pose_new.position.y =  0.05;
    pose_new.position.z = 0;
    pose_new.orientation.x=0.3826834;
    pose_new.orientation.y = 0.9238795;
    pose_new.orientation.z =  0;
    pose_new.orientation.w =  0;

    TMatrix base_to_assembly_origo(origo_mount);
    
    std::cout << pose_new<< std::endl;

    TMatrix test(pose_new);
    TMatrix inv_test = test.inverse();

    std::cout << inv_test.getPose()<< std::endl;

    TMatrix test2(pose_new);

    test2 = base_to_assembly_origo*test2;

    test2.print();
    std::cout << test2.getPose() <<std::endl;


}

AssemblyKit::~AssemblyKit()
{
}


/// PROTECTED

void AssemblyKit::init_grid(int size_y,int size_x)
{
    std::cout << "here4" << std::endl;
    std::vector<std::vector<int>> grid;
    std::vector<int> temp_grid;
    for (int i = 0; i < size_x; ++i)
    {
        temp_grid.push_back(false);
    }
    for (int i = 0; i < size_y; ++i)
    {
        grid.push_back(temp_grid);
    }
    grid_matrix = grid;
}

bool AssemblyKit::addModule(std::string _type,std::vector<Pose> _module_poses)
{
    if(true != is_kit_full)
    {
        int x;
        int y;
        // check if there already is a module at the pose
        for (int i = 0; i < _module_poses.size(); i++)
        {
            x = _module_poses[i].x;
            y = _module_poses[i].y;   
            if( grid_matrix[x][y] == true)
            {
                std::cout << "Pose is already taken" << std::endl;
                return false;
            }
        }
        // occupied the poses
        for (int i = 0; i < _module_poses.size(); i++)
        {
            x = _module_poses[i].x;
            y = _module_poses[i].y;   
            grid_matrix[x][y]=true;
        }
        Module new_module(_type,_module_poses);
        new_module.setID(id_counter);
        modules.push_back(new_module);
        
        id_counter++;
        updateGridMatrix();
        return true;
    }
    else 
    {
        return false;
    }
}

int AssemblyKit::numberOfModules()
{
    return modules.size();
}

bool AssemblyKit::removeModule(Module _module)
{
    if( _module.object_in_module == false)
    {
        int x,y;
        for (int i = 0; i < _module.poses.size(); i++)
        {
            x = _module.poses[i].x;
            y = _module.poses[i].y;   
            grid_matrix[x][y]=false;
        }
        for(int i = 0; i < modules.size(); i++)
        {
            if (_module.getID() == modules[i].getID());
            {
                modules.erase(modules.begin()+i);
                break;
            }
            if (i == modules.size())
            {
                updateGridMatrix();
                return false;
            }
        }
        updateGridMatrix();
        return true;
    }
    updateGridMatrix();
    return false;
}

int AssemblyKit::findFreeModuleIndex(std::string type)
{
    for (int i = 0; i < modules.size(); i++)
    {
        if (modules[i].type == type)
        {
            if (modules[i].object_in_module == true)
            {
                return i;
            }
        }
    }
}

void AssemblyKit::updateGridMatrix()
{
    for ( int i = 0; i < modules.size(); i++)
    {
        for (int j = 0; j < modules[i].poses.size();j++)
        {
            Pose temp_pose = modules[i].poses[j];
            if (modules[i].object_in_module == true)
            {
                grid_matrix[temp_pose.x][temp_pose.y] = 2;
            }
            else
            {
                grid_matrix[temp_pose.x][temp_pose.y] = 1;
            }
        }
    }
}


void AssemblyKit::printGrid()
{
    updateGridMatrix();
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

geometry_msgs::Pose AssemblyKit::getModulePose(int moduleIndex)
{
    TMatrix assembly_to_module;
    Pose module_coordiante = modules[moduleIndex].poses[int(((modules[moduleIndex].poses).size()-1)/2)];
    assembly_to_module.setTranslation(module_coordiante.x*0.05, module_coordiante.y*0.05, 0.0);

    TMatrix base_to_module = base_to_assembly_origo * assembly_to_module * modules[moduleIndex].dropOffPose;

    return  base_to_module.getPose();
}
