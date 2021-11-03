cmake_minimum_required(VERSION 3.0.2)
project(enviroment_controller_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


set(SOURCES
  src/main.cpp
  src/Object.cpp
  src/Module.cpp
  src/AssemblyKit.cpp
  src/EnviromentController.cpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES gripper_Cmd.msg gripper_Status.msg)


## Generate services in the 'srv' folder
add_service_files(
FILES
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
