cmake_minimum_required(VERSION 3.0)
project(pose_logger_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(pose_logger_plugin SHARED pose_logger_plugin.cpp)
target_link_libraries(pose_logger_plugin ${GAZEBO_LIBRARIES})