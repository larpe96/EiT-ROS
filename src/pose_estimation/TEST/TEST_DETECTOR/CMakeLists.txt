cmake_minimum_required(VERSION 3.5)

project(TEST_DETECTOR LANGUAGES CXX)

find_package(OpenCV REQUIRED)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(project_sources
    main.cpp
   ../../include pose_estimation/pose_estimation.hpp
   ../../src/pose_estimation.cc
)

add_executable(TEST_DETECTOR ${project_sources})
target_link_libraries(TEST_DETECTOR PRIVATE ${OpenCV_LIBS})
