cmake_minimum_required(VERSION 3.0.2)
project(enviroment_controller_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  std_srvs
  sensor_msgs
)


set(SOURCES
  src/main.cpp
  src/Module.cpp
  src/AssemblyKit.cpp
  src/EnviromentControllerNode.cpp
  src/Matrix.cpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES 
# )


add_service_files(
FILES
module_poses_srv.srv
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


#  generate_messages(
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
## Testing ##
#############

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(enviroment_controller enviroment_controller_pkg_gencpp)
