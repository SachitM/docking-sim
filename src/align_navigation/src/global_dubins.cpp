#include <pluginlib/class_list_macros.h>
 #include "global_dubins.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(dubins_planner::DubinsGlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace dubins_planner {

 DubinsGlobalPlanner::DubinsGlobalPlanner (){

 }

 DubinsGlobalPlanner::DubinsGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void DubinsGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
     if(!initialized_) {
         ROS_INFO("Reading Parameters");
         ros::NodeHandle private_nh("~/" + name);
         private_nh.param("/turning_radius", turning_radius_, 3.4);
        private_nh.param("/step_size_dubins", step_size_dubins_, 0.1);

         initialized_ = true;

     }
     

 }
 
 int DubinsGlobalPlanner::dubinsConfiguration(double q[3], double x, void* user_data) {
    // printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    struct callback_data* cb_data = static_cast<struct callback_data*>(user_data);
    geometry_msgs::PoseStamped new_goal = cb_data->dummy;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(q[2]);

    new_goal.pose.position.x = q[0];
    new_goal.pose.position.y = q[1];

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();
    cb_data->dubins_path.push_back(new_goal);
    return 0;
}

double DubinsGlobalPlanner::getYawAngle(const geometry_msgs::PoseStamped& pose_msg) {
    tf::Pose pose;
    tf::poseMsgToTF(pose_msg.pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
    return yaw_angle;
}

 bool DubinsGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    
   if (!initialized_) {
    ROS_INFO_STREAM("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
   }
    
    // dubins_path.clear();
    double roll, pitch, yaw;
    double q0[] = { start.pose.position.x, start.pose.position.y,  getYawAngle(start) };
    double q1[] = { goal.pose.position.x, goal.pose.position.y,  getYawAngle(goal) };
    double turning_radius = turning_radius_;
    DubinsPath path;
    struct callback_data cb;
    cb.dummy = goal;

    dubins_shortest_path( &path, q0, q1, turning_radius);
    
    dubins_path_sample_many( &path, step_size_dubins_, DubinsGlobalPlanner::dubinsConfiguration, static_cast<void*>(&cb));
    ROS_ERROR_STREAM("A path was planned");
    plan.insert(plan.end(), cb.dubins_path.begin(), cb.dubins_path.end());
    plan.push_back(goal);
  return true;
 }
 };