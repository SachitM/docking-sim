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


using namespace std;
class LidarDetect{

    public:
        ros::Subscriber cluster_sub;    // get centroids of clusters
        ros::ServiceServer hms_service;

        LidarDetect() {
            double buffer_time = sense_buffer + comm_buffer + safe_buffer;
            double stopping_distance = (vel * vel * 0.5 / acc) + (buffer_time * vel);

            total_distance = stopping_distance + lidar_distance + safe_distance;
        }

        //add velocity callback

        void clustering_callback(const autoware_msgs::Centroids &centroids){

            if (is_approach == true) {
                return;
            }

            double x, y;
            for (auto point : centroids.points){
                x = point.x;
                y = point.y;
                if ((std::abs(y) < width / 2) && (direction * x < total_distance) && (direction * x > lidar_distance)) {
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

        void state_callback(ADD_STATE in_state)
        {
            if (in_state->CurrState == state_machine::StateOut::State_Approach && is_approach == false)
            {
                double less_width = width / 2.0;
                update_ranges(less_width);
                is_approach = true;
            }
            else if (in_state->CurrState != state_machine::StateOut::State_Approach && is_approach == true)
            {
                update_ranges(width);
                is_approach = false;
            }
            else {
                return;
            }
        }

    private:
        bool hms_flag;
        bool is_approach;
        double width = 1.85;    // changed since 1.355 wasn not working correctly
        double vel = 4;
        double acc = 3; // assuming acc == deceleration

        // distances are in meters
        double total_distance = 5;      // to be overwritten in constructor
        double lidar_distance = 0.1;     // from the front
        double safe_distance = 0.3;

        // times are in seconds
        double sense_buffer = 1.0 / 40;    // since the lidar samples at 40 Hz
        double comm_buffer = 0.025;
        double safe_buffer = 0.1;

        int direction = 1;  // forward, use direction callback to change

};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_3d");
    ros::NodeHandle n;

    LidarDetect lid;

    lid.cluster_sub = n.subscribe("/cluster_centroids", 10, &LidarDetect::clustering_callback, &lid);

    lid.hms_service = n.advertiseService("health_check_obstacle_3d", &LidarDetect::check, &lid);

    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
