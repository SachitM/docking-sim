/*********************************************************************
* Author:  Sachit Mahajan
*********************************************************************/

#include <pluginlib/class_list_macros.h>
 #include "global_reeds_shepp.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(reeds_shepp_planner::ReedsSheppGlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace reeds_shepp_planner {

 ReedsSheppGlobalPlanner::ReedsSheppGlobalPlanner (){

 }

 ReedsSheppGlobalPlanner::ReedsSheppGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void ReedsSheppGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
     if(!initialized_) {
         ROS_INFO("Reading Parameters");
         ros::NodeHandle private_nh("~/" + name);
         private_nh.param("/turning_radius", turning_radius_, 5.0);
        private_nh.param("/step_size_reeds_shepp", step_size_reeds_shepp_, 0.1);

         initialized_ = true;

     }
     
 }
 
 int ReedsSheppGlobalPlanner::reeds_sheppConfiguration(double q[3], void* user_data) {
    struct callback_data* cb_data = static_cast<struct callback_data*>(user_data);
    geometry_msgs::PoseStamped new_goal = cb_data->dummy;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(q[2]);

    new_goal.pose.position.x = q[0];
    new_goal.pose.position.y = q[1];

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();
    cb_data->reeds_shepp_path.push_back(new_goal);
    return 0;
}

double ReedsSheppGlobalPlanner::getYawAngle(const geometry_msgs::PoseStamped& pose_msg) {
    tf::Pose pose;
    tf::poseMsgToTF(pose_msg.pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
    return yaw_angle;
}

 bool ReedsSheppGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    
   if (!initialized_) {
    ROS_INFO_STREAM("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
   }
    
    // reeds_shepp_path.clear();
    double roll, pitch, yaw;
    double q0[] = { start.pose.position.x, start.pose.position.y,  getYawAngle(start) };
    double q1[] = { goal.pose.position.x, goal.pose.position.y,  getYawAngle(goal) };
    double turning_radius = turning_radius_;
    ReedsSheppStateSpace path(turning_radius);
    struct callback_data cb;
    cb.dummy = goal;

    path.sample( q0, q1, step_size_reeds_shepp_, ReedsSheppGlobalPlanner::reeds_sheppConfiguration, static_cast<void*>(&cb));
    ROS_INFO_STREAM("[Reeds-Shepp Planner] Path planned");
    plan.insert(plan.end(), cb.reeds_shepp_path.begin(), cb.reeds_shepp_path.end());
    plan.push_back(goal);
  return true;
 }
 };