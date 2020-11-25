#include <ros/ros.h>
#include "state_machine/StateOut.h"
#include "state_machine/StateIn.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#define RAD2DEG 57.295779513

int pod_ID;
double pod_x;
double pod_y;
double pod_theta;
geometry_msgs::PoseStamped pod_msg;

bool is_active = false;

double phz_x;
double phz_y;
double phz_theta;
geometry_msgs::PoseStamped phz_msg;

// distance between pod centre and phz start
double dist = 4.0;

void set_pod_loc()
{
  pod_msg.header.frame_id = "map";
  pod_msg.pose.position.x = pod_x;
  pod_msg.pose.position.y = pod_y;
  pod_msg.pose.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, pod_theta);
  pod_msg.pose.orientation = tf2::toMsg(quat);
}

void set_phz_start()
{
  phz_theta = pod_theta;
  phz_msg.header.frame_id = "map";
  phz_msg.pose.position.x = pod_x - dist * cos(phz_theta);
  phz_msg.pose.position.y = pod_y - dist * sin(phz_theta);
  phz_msg.pose.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, phz_theta);
  phz_msg.pose.orientation = tf2::toMsg(quat);
}

void StateMachineCallback(const state_machine::StateOut::ConstPtr& msg)
{
  if ((msg->CurrState == state_machine::StateOut::State_Identify) ||
      (msg->CurrState == state_machine::StateOut::State_P2P) ||
      (msg->CurrState == state_machine::StateOut::State_D_Approach) ||
      (msg->CurrState == state_machine::StateOut::State_Verify))
  {
    is_active = true;
  }
  else
  {
    is_active = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "groundtruth_publisher_node");
  ros::NodeHandle n;

  ros::Subscriber sm_sub = n.subscribe("SM_output", 10, StateMachineCallback);
  ros::Publisher pod_gt_pub = n.advertise<geometry_msgs::PoseStamped>("pod_groundtruth", 1000);
  ros::Publisher phz_start_gt_pub = n.advertise<geometry_msgs::PoseStamped>("phz_start_groundtruth", 1000);

  ros::Rate loop_rate(1);

  // modify this to get it from pod_server
  n.getParam("/align/pod1/x_loc", pod_x);
  n.getParam("/align/pod1/y_loc", pod_y);
  n.getParam("/align/pod1/theta", pod_theta);

  while (ros::ok())
  {
    if (is_active)
    {
      set_pod_loc();
      set_phz_start();

      ROS_INFO("Pod location ground truth: X: %.2f Y: %.2f", pod_msg.pose.position.x, pod_msg.pose.position.y);
      ROS_INFO("PHZ start location ground truth: X: %.2f Y: %.2f", phz_msg.pose.position.x, phz_msg.pose.position.y);

      pod_msg.header.stamp = ros::Time::now();
      phz_msg.header.stamp = ros::Time::now();

      pod_gt_pub.publish(pod_msg);
      phz_start_gt_pub.publish(phz_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}