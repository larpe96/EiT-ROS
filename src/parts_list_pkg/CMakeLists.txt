cmake_minimum_required(VERSION 3.0.2)
project(parts_list_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_service_files(FILES missing_parts.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set( project_sources
   src/partslist.cpp
   src/parts_list_node.cpp
)


add_executable(part_list ${project_sources})
target_link_libraries(part_list ${catkin_LIBRARIES})
add_dependencies(part_list parts_list_pkg_gencpp)