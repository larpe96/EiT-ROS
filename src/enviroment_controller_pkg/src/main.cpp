#include "EnviromentControllerNode.h"

#include<sstream>
#include <signal.h>
#include <geometry_msgs/Point.h>
int run_loop = 1;

void mySigintHandler(int sig)
{
  ros::shutdown();
  run_loop = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "envir_node");
  EnviromentControllerNode node;  
  signal(SIGINT, mySigintHandler);
  ros::Rate r(1);
  while(run_loop)
  {		 ros::spinOnce();
          r.sleep();
  }
  return 0;
}