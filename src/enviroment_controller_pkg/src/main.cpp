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


// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "master");
//   EnviromentControllerNode node;
//   signal(SIGINT, mySigintHandler);
//   ros::Rate r(1);
//   while(run_loop)
//   {		 ros::spinOnce();
//           r.sleep();
//   }
//    return 0;
// };
// origoMount.orientation.x=0;
//     origoMount.orientation.y =0.;
//     origoMount.orientation.z =  -0.06014734;// 157.20 deg around base z
//     origoMount.orientation.w =  -0.99818951; // 157.20 deg around base z

int main(int argc, char **argv)
{
  std::cout <<"START" << std::endl;
  ros::init(argc, argv, "envir_node");
  EnviromentControllerNode node;  
  std::cout << "ENDED" << std::endl;
  signal(SIGINT, mySigintHandler);
  ros::Rate r(1);
  while(run_loop)
  {		 ros::spinOnce();
          r.sleep();
  }
   return 0;
  return 0;
}