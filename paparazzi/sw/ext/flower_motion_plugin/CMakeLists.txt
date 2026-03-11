cmake_minimum_required(VERSION 3.10)
project(flower_motion_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(flower_motion SHARED flower_motion_plugin.cpp)
target_link_libraries(flower_motion ${GAZEBO_LIBRARIES})