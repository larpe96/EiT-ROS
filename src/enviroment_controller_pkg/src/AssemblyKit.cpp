#include "AssemblyKit.h"

/* ======== TODO: ==============
1)  Need some logic for finding connected mounts func: check_mount_size()

*/

AssemblyKit::AssemblyKit(/* args */)
{
    is_kit_full = false;
    frame.eye();
    init_grid();
}

void AssemblyKit::test()
{


    TMatrix test(origo_mount);

    geometry_msgs::Pose p_test = test.getPose();
    std::cout<< p_test.orientation.x<<"\t"<< p_test.orientation.y<<"\t"<< p_test.orientation.z<<"\t"<< p_test.orientation.w<<"\n";
    std::cout<< p_test.position.x<<"\t"<< p_test.position.y<<"\t"<< p_test.position.z;
    std::cout<<"\n END TRANS2QUAT 1 \n\n";

    Object obj_test;

    bool b = addModule(obj_test);
    Module* _m = &mount_matrix.at(0);
}


AssemblyKit::~AssemblyKit()
{
}


/// PROTECTED

void AssemblyKit::init_grid()
{
    int len_i = sizeof(grid_matrix)/sizeof(grid_matrix[0]);
    int len_j = sizeof(grid_matrix[0])/sizeof(grid_matrix[0][0]);
    for (int i=0; i<len_i; i++)
    {
        for (int j=0; j<len_j; j++)
        {
            grid_matrix[i][j]=false;
        }
        
    }
}

bool AssemblyKit::addModule(Object _obj)
{
    if(true != is_kit_full)
    {
        Module m(_obj, frame, grip_frame);
        // if(check_mount_grid(&m))
        // {
        //     mount_matrix.push_back(m);            
        //     for(Occu_pos & mount:m.getOccuPoses())
        //         grid_matrix[mount.x][mount.y] = true; // occupied!
        //     return true;
        // }
        // return false;
    }
    else 
    {
        return false;
    }
}