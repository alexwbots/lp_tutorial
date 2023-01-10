cmake_minimum_required(VERSION 3.0.2)
project(free_local_planner_v1)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nav_core
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(free_local_planner_v1_lib src/free_planner.cpp)
