#include <pickup_db/pickup_db.hpp>


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pickup_db");
  auto nh = ros::NodeHandle("~");

  // Detector
  PickupDB pickup_database;
  pickup_database.Initialize(nh);

  // Spin
  ros::spin();
}
