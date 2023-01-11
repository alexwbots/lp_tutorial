cmake_minimum_required(VERSION 3.0.2)
project(free_local_planner_v1)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nav_core
  roscpp
  base_local_planner
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(free_local_planner_v1 src/free_planner.cpp)
add_dependencies(free_local_planner_v1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(free_local_planner_v1 ${catkin_LIBRARIES})
