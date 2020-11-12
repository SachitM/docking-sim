/*
Author: Sanil Pande
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <cmath>
#include <gazebo_msgs/ModelStates.h>
#include <autoware_msgs/Centroids.h>
#include <ros/console.h>
#include <string>
#include <limits>
#include <vector>
#include <utility>
#include <tf/transform_listener.h>
#include <hms_client/hms_msg.h>
#include <hms_client/ping_pong.h>
#include <state_machine/StateOut.h>
#include <nav_msgs/Odometry.h>


using namespace std;
class LidarDetect{

    public:
        ros::Subscriber cluster_sub;    // get centroids of clusters
        ros::ServiceServer hms_service;
        ros::Subscriber state_sub;
        ros::Subscriber vehicle_state_sub;

        LidarDetect() {
        }

        void update_ranges()
        {
            stopping_distance = (velocity * velocity * 0.5 / maximum_acceleration);
            stopping_distance += (buffer_time * velocity);
            total_distance = stopping_distance + lidar_distance + safe_distance;
        }

        void vehicle_state_callback(const nav_msgs::Odometry::ConstPtr& state)
        {
            this->velocity = state->twist.twist.linear.x;

        }

        void clustering_callback(const autoware_msgs::Centroids &centroids){

            if (is_p2p != true) {
                return;
            }

            int sign = (this->velocity) > 0 ? 1 : -1;

            double x, y;
            for (auto point : centroids.points){
                x = point.x;
                y = point.y;
                if ((std::abs(y) < width / 2) && (x * sign < total_distance) && (x * sign > lidar_distance)) {
                    ROS_INFO( "Obstacle detected!");
                    hms_flag < 1;
                    return;
                }
            }
            hms_flag = 0;
            return;
        }

        bool check(hms_client::ping_pong::Request  &req,
                 hms_client::ping_pong::Response &res)
        {
            res.msg.header.stamp = ros::Time::now();
            res.health = 1;
            res.error_code = hms_flag ? 1 : 0;
            hms_flag = false;
            return true;
        }

        void state_callback(const state_machine::StateOut::ConstPtr& in_state)
        {
            if (in_state->CurrState == state_machine::StateOut::State_P2P)
            {
                is_p2p = true;
            }
            else
            {
                is_p2p = false;
            }
        }

    private:
        bool hms_flag;
        bool is_p2p = false;

        double width = 2.0;    // Wheelbase is 1.9m + some allowance
        double velocity = 0.0;
        double maximum_acceleration = 3.0; // assuming acc == deceleration (m/s^2)

        // distances are in meters
        double total_distance = 10.0;
        double stopping_distance = 0.0;
        double lidar_distance = 0.1;     // from the front
        double safe_distance = 0.3;

        // buffer times are in seconds
        // sensing (1/sampling rate), communication, safety
        double buffer_time = (1.0 / 40) + 0.025 + 0.1;

};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_3d");
    ros::NodeHandle n;

    LidarDetect lid;

    lid.cluster_sub = n.subscribe("/cluster_centroids", 10, &LidarDetect::clustering_callback, &lid);
    lid.state_sub = n.subscribe("SM_output", 1000, &LidarDetect::state_callback, &lid);
    lid.hms_service = n.advertiseService("health_check_obstacle_3d", &LidarDetect::check, &lid);
    lid.vehicle_state_sub = n.subscribe("/ground_truth/state", 1000, &LidarDetect::vehicle_state_callback, &lid);

    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
