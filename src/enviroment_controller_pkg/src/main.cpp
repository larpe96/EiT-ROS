#include "EnviromentControllerNode.h"
// #include<sstream>
// #include <signal.h>
#include <geometry_msgs/Point.h>
// int run_loop = 1;

// void mySigintHandler(int sig)
// {
//   ros::shutdown();
//   run_loop = 0;
// }


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


// init_assKit_pose.orientation.x = 0.573358194501323;
// init_assKit_pose.orientation.y = 0.8192435837733383;
// init_assKit_pose.orientation.z = 0.005706168482240197;
// init_assKit_pose.orientation.w = 0.008232307431875443;

int main(int argc, char **argv)
{
  geometry_msgs::Point init_pos;
  init_pos.x = 0.34496893818469754;
  init_pos.y = -0.17792548888661716;
  init_pos.z = -0.012063987217307198;

  ros::init(argc, argv, "envir_node");
  EnviromentControllerNode node(init_pos);  
  //ros::shutdown();
  return 0;
}