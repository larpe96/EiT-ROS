#include "EnviromentControllerNode.h"
#include<sstream>
#include <signal.h>

int run_loop = 1;

void mySigintHandler(int sig)
{
  ros::shutdown();
  run_loop = 0;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "master");
  EnviromentControllerNode node;
  signal(SIGINT, mySigintHandler);
  ros::Rate r(1);
  while(run_loop)
  {		 ros::spinOnce();
          node.stateLoop();
          r.sleep();
  }
   return 0;
};
