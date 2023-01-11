#ifndef FREE_PLANNER_H_
#define FREE_PLANNER_H_

#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>

#include <time.h>
#include <fstream>
#include <iostream>
using namespace std;

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes 
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// other
#include <array>
#include <vector>

namespace free_local_planner_v1{
  
  class LocalPlanner : public nav_core::BaseLocalPlanner {
  
    public:
      
      LocalPlanner();
      LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      ~LocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      bool isGoalReached();
    
    private:
    
      costmap_2d::Costmap2DROS* costmap_ros_;
      tf2_ros::Buffer* tf_;
      bool initialized_;

      // Save the path global
      std::vector<geometry_msgs::PoseStamped> global_path_;

      // Pose of the robot
      bool goal_reached;

      // Node handler for the parameters
      ros::NodeHandle nh;
  };
};

#endif
