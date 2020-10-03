 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include "dubins.h"
 #include <tf/tf.h>

 using std::string;

 #ifndef DUBINS_PLANNER_CPP
 #define DUBINS_PLANNER_CPP

 namespace dubins_planner {

 struct callback_data{
   geometry_msgs::PoseStamped dummy;
   std::vector<geometry_msgs::PoseStamped> dubins_path;
 };

 class DubinsGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:

  DubinsGlobalPlanner();
  DubinsGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
  static int dubinsConfiguration(double q[3], double x, void* user_data);
  
 private:
  bool initialized_ = false;
  double turning_radius_; 
  double step_size_dubins_;
  // std::vector<geometry_msgs::PoseStamped> dubins_path;
//   std::vector<double> get_dubins_path();
  
   double getYawAngle(const geometry_msgs::PoseStamped& pose_msg);
  };
 };
 #endif