cmake_minimum_required(VERSION 3.0.2)
project(master_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pose_estimation
  position_controller_pkg
  enviroment_controller_pkg
  parts_list_pkg
  ur_robot_pkg
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  std_srvs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES gripper_Cmd.msg gripper_Status.msg)


## Generate services in the 'srv' folder
add_service_files(
FILES
system_state_srv.srv
gripper_Conf.srv 
gripper_Incr.srv 
gripper_Move.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

 generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
   sensor_msgs
   geometry_msgs
 )
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES master_pkg
#  CATKIN_DEPENDS pose_estimation position_controller_pkg roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(INCLUDE_DIR /src)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)


#############
## Install ##
#############



#############
## Testing ##
#############

add_executable(master src/main.cpp src/MasterNode.cpp)
target_link_libraries(master ${catkin_LIBRARIES})
#add_dependencies(master master_pkg_gencpp)
