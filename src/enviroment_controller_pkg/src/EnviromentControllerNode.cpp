#include "EnviromentControllerNode.h"



/*--------------------------------------------------------------------
 * Constructor.
 *------------------------------------------------------------------*/
EnviromentControllerNode::EnviromentControllerNode()
{
    resetAssemblyKit();
    //server services
    reset_kit_server = n.advertiseService("reset_kit", &EnviromentControllerNode::resetKit,this);
    get_module_poses_server = n.advertiseService("get_module_poses", &EnviromentControllerNode::sendModulePoses,this);
} // end 

void EnviromentControllerNode::resetAssemblyKit()
{
    assembly_kit = loadAssemblyKit(jsonpath,"assemblyKit.json");
    loadModules(jsonpath,"module.json");
}

bool EnviromentControllerNode::resetKit(std_srvs::Trigger::Request  &req,
                                std_srvs::Trigger::Response &res)
{
  resetAssemblyKit();
  res.success = 1;
  return true;
}

bool EnviromentControllerNode::sendModulePoses(enviroment_controller_pkg::module_poses_srv::Request  &req,
                                enviroment_controller_pkg::module_poses_srv::Response &res)
{
    geometry_msgs::Pose drop_off_pose;
    geometry_msgs::Pose approach_pose;
    res.success = false;
    int module_index = assembly_kit.findFreeModuleIndex(req.obj_type);
    
    if( module_index != -1)
    {
        drop_off_pose = assembly_kit.getModuleDropOffPose(module_index);
        approach_pose = assembly_kit.getModuleApproachPose(module_index);
        res.success = true;
        assembly_kit.setModuleoccupied(module_index);
    }

    res.drop_off_pose = drop_off_pose;
    res.approach_pose = approach_pose;
    
    return true;
}


/*--------------------------------------------------------------------
 * Destructor.
 *------------------------------------------------------------------*/

EnviromentControllerNode::~EnviromentControllerNode()
{

} // end

std::vector<std::vector<int>> EnviromentControllerNode::getGridmatrix(std::string grid)
{
    //convert a string, ex "0,0;0,1;", to a matrix of ints, ex. [[0,0],[0,1]]

    std::vector<std::vector<int>> grid_matrix;
    std::vector<std::string> temp_vec_string;

    size_t pos = 0;
    std::string delimiter = ";";

    std::string token;

    while ((pos = grid.find(delimiter)) != std::string::npos) {
        token = grid.substr(0, pos);
        temp_vec_string.push_back(token);
        grid.erase(0, pos + delimiter.length());
    }
    
    delimiter = ',';
    
    for (int i = 0; i < temp_vec_string.size(); i++)
    {
        std::vector<int> temp_vec;
        while ((pos = temp_vec_string[i].find(delimiter)) != std::string::npos) {
            token = temp_vec_string[i].substr(0, pos);
            temp_vec.push_back(stoi(token));
            temp_vec_string[i].erase(0, pos + delimiter.length());
        }
        grid_matrix.push_back(temp_vec);
    }
    return grid_matrix;
}
AssemblyKit EnviromentControllerNode::loadAssemblyKit(std::string jsonpath,std::string filename)
{
    pt::ptree root;
    
    pt::read_json(jsonpath + filename, root);  

    string  kit = root.get<string>("kit"); 

    std::vector<std::vector<int>> grid_matrix = getGridmatrix(kit);
    pt::ptree kit_pose = root.get_child("kit_pose");
    pt::ptree trans = kit_pose.get_child("Translation");
    pt::ptree ori = kit_pose.get_child("Orientation");

    geometry_msgs::Pose base_to_kit_origo;    
    base_to_kit_origo.position.x = trans.get<float>("x");
    base_to_kit_origo.position.y = trans.get<float>("y");
    base_to_kit_origo.position.z = trans.get<float>("z");
    base_to_kit_origo.orientation.x = ori.get<float>("x");
    base_to_kit_origo.orientation.y = ori.get<float>("y");
    base_to_kit_origo.orientation.z = ori.get<float>("z");
    base_to_kit_origo.orientation.w = ori.get<float>("w");

    AssemblyKit loaded_assembly(grid_matrix,base_to_kit_origo);
    return loaded_assembly;
}

void EnviromentControllerNode::loadModules(std::string jsonpath,std::string filename)
{
    for (int x = 0; x < assembly_kit.grid_matrix.size(); x++)
    {
        for (int y = 0; y < assembly_kit.grid_matrix[0].size(); y++)
        {
            if (assembly_kit.grid_matrix[x][y] != 0)
            {
                Pose module_position;
                module_position.x = x;
                module_position.y = y;
                assembly_kit.modules.push_back(loadModule(module_position,assembly_kit.grid_matrix[x][y], jsonpath, filename));
            }   
        }
    }
}
Module EnviromentControllerNode::loadModule(Pose module_position,int module_number ,std::string jsonpath,std::string filename)
{
    pt::ptree root;

    pt::read_json(jsonpath + filename, root);  

    pt::ptree modules = root.get_child("modules");
    pt::ptree specific_module = modules.get_child(to_string(module_number));
    std::string module_type = specific_module.get<std::string>("module_type");

    pt::ptree module_dropoff = specific_module.get_child("module_dropoff");
    pt::ptree trans = module_dropoff.get_child("Translation");
    pt::ptree ori = module_dropoff.get_child("Orientation");

    geometry_msgs::Pose dropoff_pose;    
    dropoff_pose.position.x = trans.get<float>("x");
    dropoff_pose.position.y = trans.get<float>("y");
    dropoff_pose.position.z = trans.get<float>("z");
    dropoff_pose.orientation.x = ori.get<float>("x");
    dropoff_pose.orientation.y = ori.get<float>("y");
    dropoff_pose.orientation.z = ori.get<float>("z");
    dropoff_pose.orientation.w = ori.get<float>("w");


    pt::ptree module_approach = specific_module.get_child("module_approach_pose");
    trans = module_approach.get_child("Translation");
    ori = module_approach.get_child("Orientation");

    geometry_msgs::Pose approach_pose;    
    approach_pose.position.x = trans.get<float>("x");
    approach_pose.position.y = trans.get<float>("y");
    approach_pose.position.z = trans.get<float>("z");
    approach_pose.orientation.x = ori.get<float>("x");
    approach_pose.orientation.y = ori.get<float>("y");
    approach_pose.orientation.z = ori.get<float>("z");
    approach_pose.orientation.w = ori.get<float>("w");

    // for (pt::ptree::value_type& module : root.get_child("modules"))
    // {
    //     std::cout << "HELLO3" << std::endl;
    //     test2 = test1.get_child(module.first);
    //     std::cout << test2.get<std::string>("module_type") << std::endl;
    //     geometry_msgs::Pose dropoff_pose;    
    //     dropoff_pose.position.x = trans.get<float>("x");
    //     dropoff_pose.orientation.x = ori.get<float>("x");
    //     std::cout << dropoff_pose << std::endl;
    // }

    Module module(0, module_type,module_position, dropoff_pose,approach_pose);
    return module;
}
