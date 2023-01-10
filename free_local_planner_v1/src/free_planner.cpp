#include <free_local_planner_v1/free_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

PLUGINLIB_EXPORT_CLASS(free_local_planner_v1::LocalPlanner, nav_core::BaseLocalPlanner)

namespace free_local_planner_v1{

  LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

  LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
      initialize(name, tf, costmap_ros);
    }

  LocalPlanner::~LocalPlanner() {}

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
}
