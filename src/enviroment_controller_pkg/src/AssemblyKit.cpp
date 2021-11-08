#include "AssemblyKit.h"

/* ======== TODO: ==============
1)  Need some logic for finding connected mounts func: check_mount_size()

*/

AssemblyKit::AssemblyKit(/* args */)
{
    isKitFull = false;
    frame.eye();
    init_occuGrid();
    // mat.print();
    // TMatrix a(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15);
    // a.print();    
    // TMatrix b(1,2,3,4,0,0,0,0,0,0,0,0,0,0,0,1);
    // b.print();
    // (b*a).print();

    // geometry_msgs::Point p = (b*a).getPos();

    // std::cout<<p.x<< "\n"<<p.y<< "\n"<<p.z<<std::endl;

    
}


void AssemblyKit::setOrigoMount(geometry_msgs::Point _pos)
{
    setOrientation();
    origoMount.position = _pos;
    
    origoGripPose.position.x = 0;
    origoGripPose.position.y = 0;
    origoGripPose.position.z = 0;

    
    origoGripPose.orientation.x = 0.9811683600184548;
    origoGripPose.orientation.y = -0.18890178777094666;
    origoGripPose.orientation.z = 0.030302333715146366;
    origoGripPose.orientation.w = 0.026580678077361676;

    TMatrix temp(origoMount);
    TMatrix grip(origoGripPose);
    gripFrame = grip;
    frame = temp;
    test();


}

void AssemblyKit::test()
{


    TMatrix test(origoMount);
    // std::cout<<"heyo\n\n";
    // test.print();
    // std::cout<<"end\n\n INVSERSE \n\n";
    // test.inverse().print();

    geometry_msgs::Pose p_test = test.getPose();
    std::cout<< p_test.orientation.x<<"\t"<< p_test.orientation.y<<"\t"<< p_test.orientation.z<<"\t"<< p_test.orientation.w<<"\n";
    std::cout<< p_test.position.x<<"\t"<< p_test.position.y<<"\t"<< p_test.position.z;
    std::cout<<"\n END TRANS2QUAT 1 \n\n";

    Object obj_test;

    bool b = add_module(obj_test);
    Module* _m = &mountMatrix.at(0);
    geometry_msgs::Pose test_b = _m->getPlacePose();
    std::cout<<"\n\n NEW Module Pose \n\n";
    std::cout<< test_b.orientation.x<<"\t"<< test_b.orientation.y<<"\t"<< test_b.orientation.z<<"\t"<< test_b.orientation.w<<"\n";
    std::cout<< test_b.position.x<<"\t"<< test_b.position.y<<"\t"<< test_b.position.z;
    std::cout<<"\n END NEW Module Pose\n\n";
}


AssemblyKit::~AssemblyKit()
{
}


/// PROTECTED

void AssemblyKit::init_occuGrid()
{
    int len_i = sizeof(occuGrid)/sizeof(occuGrid[0]);
    int len_j = sizeof(occuGrid[0])/sizeof(occuGrid[0][0]);
    for (int i=0; i<len_i; i++)
    {
        for (int j=0; j<len_j; j++)
        {
            occuGrid[i][j]=false;
        }
        
    }
}

void AssemblyKit::increase_nect_empt()
{
    next_empty.x++;
    while(occuGrid[next_empty.x][next_empty.y])
    {
        if ( next_empty.x >=dim[0])
        {
            next_empty.x =0;
            next_empty.y++;
        }
        else
            next_empty.x++;
        if(next_empty.x >=dim[0] && next_empty.y >=dim[1])
        {
            isKitFull = true;
        }
    }
}

bool AssemblyKit::check_mount_grid(Module* _m)
{
    Occu_pos mount_ptr =  next_empty;
    int no_connected_mounts = _m->getSize();
    std::vector<Occu_pos> solution;
    while(no_connected_mounts > 0)
    {
        if(!occuGrid[mount_ptr.x][mount_ptr.y])
        {
            no_connected_mounts--;
            solution.push_back(mount_ptr);
        }
        else{ // clear if no connected solution is found in that row/col
            solution.clear();
        }
        // Need some logic for finding connected mounts
    }
    if(solution.size() == _m->getSize())
    {
        _m->setOccuPoses(solution);
        return true;
    }

}

bool AssemblyKit::add_module(Object _obj)
{
    if(true != isKitFull)
    {
        Module m(_obj, frame, gripFrame);
        if(check_mount_grid(&m))
        {
            mountMatrix.push_back(m);            
            for(Occu_pos & mount:m.getOccuPoses())
                occuGrid[mount.x][mount.y] = true; // occupied!
            increase_nect_empt();
            return true;
        }
        return false;
    }
    else 
    {
        return false;
    }
}


void AssemblyKit::setOrientation()
{
    origoMount.orientation.x=0;
    origoMount.orientation.y =0.;
    origoMount.orientation.z =  -0.06014734;// 157.20 deg around base z
    origoMount.orientation.w =  -0.99818951; // 157.20 deg around base z
}