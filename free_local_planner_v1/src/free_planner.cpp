#include <free_local_planner_v1/free_planner.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_core/parameter_magic.h>

PLUGINLIB_EXPORT_CLASS(free_local_planner_v1::LocalPlanner, nav_core::BaseLocalPlanner)

namespace free_local_planner_v1
{

  LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), global_path_(NULL)
  {
  }

  LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false), global_path_(NULL), goal_reached(false)
    {
      initialize(name, tf, costmap_ros);
    }

  LocalPlanner::~LocalPlanner() 
  {
  }

  void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(!initialized_)
    {
      costmap_ros_ = costmap_ros;
      tf_ = tf;
     
      // set initialized flag
      initialized_ = true;

      // this is only here to make this process visible in the rxlogger right from the start
      ROS_DEBUG("Free Local Planner plugin initialized.");
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    return true;
  }
  
  bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    return true;
  }
  
  bool LocalPlanner::isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
  return goal_reached;
  }
}
