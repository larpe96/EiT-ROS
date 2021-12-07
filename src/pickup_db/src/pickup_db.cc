#include <pickup_db/pickup_db.hpp>

PickupDB::PickupDB(): nh_("~")
{}

void PickupDB::Initialize(const ros::NodeHandle& nh)
{
  // Load JSON file
  LoadJson("../workspace/src/pickup_db/etc/objetcs.json");

  // Node handler
  nh_ = nh;
  service_ = nh_.advertiseService("/pickup_db", &PickupDB::OnPose, this);
}

bool PickupDB::OnPose(pickup_db::pickup_db_srv::Request &req,
                        pickup_db::pickup_db_srv::Response &res)
{
  res.pose_pickup = GetPickupPose(req.object_type, req.object_pose);
  res.pose_approach = GetApproachPose(res.pose_pickup);
  res.open_width = GetOpenWidth(req. object_type);

  return true;
}

geometry_msgs::Pose PickupDB::GetPickupPose(std::string object_type, geometry_msgs::Pose object_pose)
{
  geometry_msgs::Pose pose_pickup = object_pose;
  pt::ptree translation = object_types.get_child(object_type).get_child("Translation");

  pose_pickup.position.z = translation.get<float>("z");

  return pose_pickup;
}

geometry_msgs::Pose PickupDB::GetApproachPose(geometry_msgs::Pose pose_pickup)
{
  geometry_msgs::Pose pose_approach = pose_pickup;
  pose_approach.position.z += 0.1; // [m]

  return pose_approach;
}

float PickupDB::GetOpenWidth(std::string object_type)
{
  return object_types.get_child(object_type).get_child("Gripper").get<float>("open_width");
}

void PickupDB::LoadJson(std::string path)
{
  pt::read_json(path, root);

  object_types = root.get_child("object_types");
}