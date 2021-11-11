#include "EnviromentControllerNode.h"



EnviromentControllerNode::EnviromentControllerNode()
{
    geometry_msgs::Point pos;
    pos.x =1;
    pos.y = 2;
    pos.z=3;
}



/*--------------------------------------------------------------------
 * masterNode()
 * Constructor.
 *------------------------------------------------------------------*/
EnviromentControllerNode::EnviromentControllerNode(geometry_msgs::Point _oriPose)
{

} // end masterNode()
geometry_msgs::Point EnviromentControllerNode::convert_kit_to_base_pose(geometry_msgs::Point)
{   
    geometry_msgs::Point x;
    return  x;
}
/*--------------------------------------------------------------------
 * ~masterNode()
 * Destructor.
 *------------------------------------------------------------------*/

EnviromentControllerNode::~EnviromentControllerNode()
{

} // end ~masterNode()
