#include <ros/ros.h>
#include <hms_client/hms_msg.h>
#include <hms_client/ping_pong.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h> 
#include <cstdlib>
#include <cctype>
#include <string>
#include "std_msgs/String.h"
using namespace std;

int counter = 0;
vector<int> flags;
float prevx = 0.0, prevy = 0.0;
int ndt_flag = 0;
int scan_steps = 560;
void PointCloud2_callback(const sensor_msgs::PointCloud2& msg)//1
{
   if(!msg.is_dense)
    ROS_INFO("Recieved valid pcd data");
   else
    ROS_WARN("Received invalid pcd data");
   counter++;
   flags[0] = 1;
   //sleep(10);
}
//2
void LaserScan_callback(const sensor_msgs::LaserScan& msg)
{
   int min_ = msg.range_min - 0.1;
   int max_ = msg.range_max + 0.1;
   int flag = 1;
   for(int h = 0; h < scan_steps; h++)
    if(!isinf(msg.ranges[h]))
      if(msg.ranges[h] < 0)
        flag = 0;
        
    //if(msg.ranges[h] > max_ || msg.ranges[h] < min_ || msg.ranges[h] == INT_MAX)
      //if(msg.ranges[h] <= 0)
   if(!flag)
    ROS_ERROR("Received invalid laser scan data");
   else 
    ROS_INFO("Received valid laser scan data");
   counter++;
   flags[1] = 1;
   //sleep(10);
}
void ndt_pose_callback(const geometry_msgs::PoseStamped& msg)//3
{
   float xdiff = float(prevx - msg.pose.position.x);
   float ydiff = float(prevy - msg.pose.position.y);
   float dist_sq = (xdiff * xdiff) + (ydiff * ydiff);
  
   if(int(dist_sq) > 100)
   {
    ROS_WARN("New %f , %f Old %f , %f", msg.pose.position.x,msg.pose.position.y,prevx,prevy);
    ROS_ERROR("[NDT]Bot instructed to come to halt, since sanity check failed");
    ndt_flag = 1;
   }
   else
   {
    ROS_INFO("Received valid ndt pose data");
   }
   counter++;
   prevx = msg.pose.position.x + 0;
   prevy = msg.pose.position.y + 0;
   flags[2] = 1;

   //sleep(10);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hms_client");
  ros::NodeHandle nh;

  vector<ros::Subscriber> subs;
  vector<ros::ServiceClient> clients;
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  

  int cb_queue = 1;
  int loop_count = 0;
  

  int num_nodes;
  string str, nodei;
  vector<string> nodes;
  ros::param::get("num_nodes", num_nodes);
  for(int i = 0; i < num_nodes; i++)
  {
    string hc = "health_check_";
    str = "node" + to_string(i + 1);
    ros::param::get(str, nodei);
    //cout << nodei.c_str(); 
    nodes.push_back(nodei.c_str());
    hc.append(nodei);
    clients.push_back(nh.serviceClient<hms_client::ping_pong>(hc));

  }

  vector<string> topics;
  vector<float> rates;
  int num_topics;
  string topici, str_, r;
  float ri;
  ros::param::get("num_topics", num_topics);
  for(int i = 0; i < num_topics; i++)
  {
    str_ = "topic" + to_string(i + 1);
    r = "rate" + to_string(i + 1);
    ros::param::get(str_, topici);
    ros::param::get(r, ri);
    topics.push_back(topici.c_str());
    rates.push_back(ri);
    flags.push_back(0);
    //cout << topici.c_str(); 
    switch(i)//////////////////////////////////////////////////////for now redundant///////////////////////////////////////
    {
      case 0: subs.push_back(nh.subscribe(topics[i], cb_queue, PointCloud2_callback));
              break;
      case 1: subs.push_back(nh.subscribe(topics[i], cb_queue, LaserScan_callback));
              break;
      case 2: subs.push_back(nh.subscribe(topics[i], cb_queue, ndt_pose_callback));
              break;
    }
  }

  float lr;
  ros::param::get("min_rate", lr);
  lr = min(lr, float(1));
  cout << lr;
  ros::Rate loop_rate(1);
  //ros::MultiThreadedSpinner spinner(2);

  while(ros::ok()) {
    cout << endl;
    counter = 0;
    string nodem = "obstacle_2d";
    for(int i = 0; i < num_topics; i++)
      flags[i] = 0;
    ros::spinOnce();

    if((ndt_flag == 1) && (loop_count != 0))
    {
      geometry_msgs::Twist msg;
      msg.linear.x = 0;
      vel_pub.publish(msg);
    }
    //spinner.spin();
    //cout << counter;
    float check = 0;
    for(auto r: rates)
      if(r > lr)
        check += (r / lr);
      else
        check += 1;
    
    //cout << counter << endl;
    if(cb_queue == 1)
      check = num_topics;
    if(counter == check)
      ROS_INFO("Persistence check passed [laser, velodyne and ndt_pose]");
    else
    {
      for(int m = 0; m < num_topics; m++)
      {
        if(!flags[m])
        {
          ROS_ERROR("Failed to recieve data from topic %s ", topics[m].c_str());
        }

      }
      ROS_INFO("Persistence check [PointCloud %d, Lidar %d, NDT: %d]", flags[0],flags[1],flags[2]);
    }

    for(int j = 0; j < num_nodes; j++)
    {
      hms_client::ping_pong srv;
      srv.request.node_name = nodes[j];
      bool val = 0;
      for(int k = 0; k < 3 && val == 0; k++)
        val = clients[j].call(srv);
      if(val)
      {
        counter += 1;
        if(srv.response.error_code)//obstacle detected..
        {
          ROS_ERROR("Error : [Obstacle detected], [Node %s] Vehicle instructed to come to halt", nodes[j].c_str());
          geometry_msgs::Twist _msg;
          _msg.linear.x = 0;
          vel_pub.publish(_msg);
        }
        else
          ROS_INFO("Node %s is functioning properly", nodes[j].c_str());
      }
      else
      {
        ROS_ERROR("Failed to call service, node %s malfunctioning", srv.request.node_name.c_str());
        //ROS_ERROR("!!!Failed to call service, node malfunctioning!!!");
        if(nodes[j].c_str() == nodem)
        {
          geometry_msgs::Twist twist_msg;
          twist_msg.linear.x = 0;
          vel_pub.publish(twist_msg);
          ROS_ERROR("[Node %s] Vehicle instructed to come to halt", nodes[j].c_str());
        }
      }
    }

    ros::Publisher hms_status_pub = nh.advertise<std_msgs::String>("HMS_Status", 1);
    if(counter == num_nodes + num_topics)
    {
      std_msgs::String msg;
      msg.data = "Passed";
      hms_status_pub.publish(msg);
    }
    else
    {
      std_msgs::String msg;
      msg.data = "Failed";
      hms_status_pub.publish(msg);
    }
    loop_count += 1;
    loop_rate.sleep();

  }  
  return 0;
}