#include "EnviromentControllerNode.h"



EnviromentControllerNode::EnviromentControllerNode()
{
    geometry_msgs::Point pos;
    pos.x =1;
    pos.y = 2;
    pos.z=3;
    assemKit.setOrigoMount(pos);
}



/*--------------------------------------------------------------------
 * masterNode()
 * Constructor.
 *------------------------------------------------------------------*/
EnviromentControllerNode::EnviromentControllerNode(geometry_msgs::Point _oriPose)
{
    assemKit.setOrigoMount(_oriPose);

} // end masterNode()

/*--------------------------------------------------------------------
 * ~masterNode()
 * Destructor.
 *------------------------------------------------------------------*/

EnviromentControllerNode::~EnviromentControllerNode()
{

} // end ~masterNode()
