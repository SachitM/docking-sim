 /*********************************************************************
* Author:  Sachit Mahajan
*********************************************************************/
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include "reeds_shepp.h"
 #include <tf/tf.h>

 using std::string;

 #ifndef REEDS_SHEPP_PLANNER_CPP
 #define REEDS_SHEPP_PLANNER_CPP

 namespace reeds_shepp_planner {

 struct callback_data{
   geometry_msgs::PoseStamped dummy;
   std::vector<geometry_msgs::PoseStamped> reeds_shepp_path;
 };

 class ReedsSheppGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:

  ReedsSheppGlobalPlanner();
  ReedsSheppGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
  static int reeds_sheppConfiguration(double q[3], void* user_data);
  
 private:
  bool initialized_ = false;
  double turning_radius_; 
  double step_size_reeds_shepp_;
  
   double getYawAngle(const geometry_msgs::PoseStamped& pose_msg);
  };
 };
 #endif