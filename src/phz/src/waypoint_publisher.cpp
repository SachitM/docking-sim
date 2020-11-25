#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <vector>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define RAD2DEG 57.295779513
#define AVERAGE_PRED 0

// std::tuple<double, double, geometry_msgs::Quaternion> gt_coords;
std::tuple<double, double, geometry_msgs::Quaternion> pred_coords;
std::vector<double> last_pred(3, 0);

int count = 0;
int flag_pred = 0;
// int flag_gt = 0;

// functions

std::vector<double> set_params(const geometry_msgs::PoseStamped::ConstPtr& msg);
std::vector<geometry_msgs::Pose> generate_wps(double x, double y, geometry_msgs::Quaternion q);
geometry_msgs::Pose set_pose(double R, double theta, geometry_msgs::Quaternion q);

/*
void pod_gt_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
  std::get<0>(gt_coords) = msg->pose.position.x;
  std::get<1>(gt_coords) = msg->pose.position.y;
  std::get<2>(gt_coords) = msg->pose.orientation;

  flag_gt = 1;
}
*/

void pod_pred_CB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::vector<double> v = set_params(msg);
  last_pred[0] = (last_pred[0] * count + v[0]) / (count + 1);
  last_pred[1] = (last_pred[1] * count + v[1]) / (count + 1);
  last_pred[2] = (last_pred[2] * count + v[2]) / (count + 1);

  if (AVERAGE_PRED == 1)
    count++;

  std::get<0>(pred_coords) = last_pred[0];
  std::get<1>(pred_coords) = last_pred[1];

  tf2::Quaternion quat;
  quat.setRPY(0, 0, last_pred[2]);

  std::get<2>(pred_coords) = tf2::toMsg(quat);

  flag_pred = 1;
}

double quat_to_yaw(double x, double y, double z, double w)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

std::vector<double> set_params(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::vector<double> v(3, 0.0);
  v[0] = msg->pose.position.x;
  v[1] = msg->pose.position.y;
  v[2] =
      quat_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  return v;
}

geometry_msgs::Pose set_pose(double x, double y, double d, geometry_msgs::Quaternion q, double theta)
{
  geometry_msgs::Pose p;
  p.position.x = x - d * cos(theta);
  p.position.y = y - d * sin(theta);
  p.position.z = 0;
  p.orientation = q;
  return p;
}

std::vector<geometry_msgs::Pose> generate_wps(double x, double y, geometry_msgs::Quaternion q)
{
  std::vector<geometry_msgs::Pose> poses;
  double theta = quat_to_yaw(q.x, q.y, q.z, q.w);

  poses.push_back(set_pose(x, y, 3.5, q, theta));
  poses.push_back(set_pose(x, y, 3.0, q, theta));
  poses.push_back(set_pose(x, y, 2.0, q, theta));
  poses.push_back(set_pose(x, y, 1.0, q, theta));
  poses.push_back(set_pose(x, y, 0, q, theta));

  return poses;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_publisher_node");
  ros::NodeHandle n;

  ros::Subscriber pod_pred_sub = n.subscribe("pod_predicted_laser", 1000, pod_pred_CB);
  // ros::Subscriber pod_gt_sub = n.subscribe("pod_groundtruth",1000,pod_gt_CB);

  ros::Publisher wp_pred_pub = n.advertise<geometry_msgs::PoseArray>("waypoints_goal", 10);
  // ros::Publisher wp_gt_pub = n.advertise<geometry_msgs::PoseArray>("gt_waypoints_goal",10);

  geometry_msgs::PoseArray wp_pred;
  // geometry_msgs::PoseArray wp_gt;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (flag_pred)
    {
      wp_pred.header.stamp = ros::Time::now();
      wp_pred.header.frame_id = "map";
      wp_pred.poses = generate_wps(std::get<0>(pred_coords), std::get<1>(pred_coords), std::get<2>(pred_coords));
      wp_pred_pub.publish(wp_pred);
      flag_pred = 0;

      ROS_INFO("Yaw: %.2f", last_pred[2] * RAD2DEG);
      ROS_INFO("WP_0: X: %.2f, Y: %.2f", wp_pred.poses[0].position.x, wp_pred.poses[0].position.y);
      ROS_INFO("WP_1: X: %.2f, Y: %.2f", wp_pred.poses[1].position.x, wp_pred.poses[1].position.y);
      ROS_INFO("WP_2: X: %.2f, Y: %.2f", wp_pred.poses[2].position.x, wp_pred.poses[2].position.y);
      ROS_INFO("WP_3: X: %.2f, Y: %.2f", wp_pred.poses[3].position.x, wp_pred.poses[3].position.y);
      ROS_INFO("WP_4: X: %.2f, Y: %.2f", wp_pred.poses[4].position.x, wp_pred.poses[4].position.y);
    }

    /*
    if (flag_gt){
      wp_gt.header.stamp = ros::Time::now();
      wp_gt.header.frame_id = "map";
      wp_gt.poses = generate_wps(std::get<0>(gt_coords),std::get<1>(gt_coords),std::get<2>(gt_coords));
      wp_gt_pub.publish(wp_gt);
      flag_gt = 0;
    }
    */
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}