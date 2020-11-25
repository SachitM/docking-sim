/*
Author: Sanil Pande
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <hms_client/hms_msg.h>
#include <hms_client/ping_pong.h>
#include <math.h>
#include <state_machine/StateOut.h>
#include <nav_msgs/Odometry.h>

#define RANGE 1.22172999382         // angular range
#define INCREMENT 0.0043711271137   // angular increment between laser readings
#define LASER_STEPS 560

class Chassis {
    public:
        ros::Subscriber lidar_sub;
        ros::Subscriber state_sub;
        ros::Subscriber vehicle_state_sub;

        ros::Publisher obstacle_pub;
        ros::ServiceServer hms_service;

        Chassis() {            
            update_ranges(width);
        }

        void obstacle_detection_callback(const sensor_msgs::LaserScan scan)
        {
            if (is_approach != true && is_p2p != true && is_emergency != true) {
                return;
            }
            // iterating over all values since ideally all element in range should be non zero
            for (int i = 0; i < LASER_STEPS; i++) {
                if (range_array[i] > scan.ranges[i]) {
                    if (!hms_flag) {
                       ROS_INFO( "Obstacle detected!");
                    }
                    hms_flag = true;
                    return;
                }
            }
            if (hms_flag)
            {
                hms_flag = false;
                ROS_INFO( "Obstacle removed!");
            }
        }

        
        
        void vehicle_state_callback(const nav_msgs::Odometry::ConstPtr& state)
        {
            this->velocity = state->twist.twist.linear.x;

        }

        bool check(hms_client::ping_pong::Request  &req,
                 hms_client::ping_pong::Response &res)
        {
            res.msg.header.stamp = ros::Time::now();
            res.health = 1;
            res.error_code = hms_flag ? 1 : 0;
            //hms_flag = false;
            return true;
        }

        void state_callback(const state_machine::StateOut::ConstPtr& in_state)
        {
            if (in_state->CurrState == state_machine::StateOut::State_D_Approach)
            {
                double less_width = width / 1.5;
                update_ranges(less_width);
                ROS_INFO( "Beginning Approach - Constraining 2D obstacle detection width");
                is_p2p = false;
                is_approach = true;
                is_emergency = false;
            }
            else if (in_state->CurrState == state_machine::StateOut::State_P2P)
            {
                update_ranges(width);
                ROS_INFO( "Using original 2D obstacle detection width.");
                is_approach = false;
                is_p2p = true;
                is_emergency = false;
            }
            else if (in_state->CurrState == state_machine::StateOut::State_EHS){
                is_approach = false;
                is_p2p = false;
                is_emergency = true;
            }
            else {
                is_approach = false;
                is_p2p = false;
                is_emergency = false;
                return;
            }
        }

        void update_ranges(double width)
        {
            stopping_distance = (velocity * velocity * 0.5 / maximum_acceleration);
            stopping_distance +=  + (buffer_time * velocity);
            total_distance = stopping_distance + lidar_distance + safe_distance;

            // if (total_distance < 0.5) {
            //     total_distance = 0.5;   // less than this will throw errors, out of range of scan beams
            // }
            
            double angle = atan2(width, 2 * total_distance);

            double start = (RANGE - angle) / INCREMENT;
            int s = int(start);

            s = std::max(std::min(s, LASER_STEPS - 1), 0);

            // need to add code for blind spot ditances
            for (int i = s; i < LASER_STEPS - s; i++) {
                range_array[i] = total_distance;
            }
        }


    private:
        bool hms_flag = false;
        bool is_approach = false;
        bool is_p2p = false;
        bool is_emergency = false;
        double range_array[LASER_STEPS] = {0};

        double width = 2.0;    // Wheelbase is 1.9m + some allowance
        double velocity = 0.0;
        double maximum_acceleration = 1.0;//3.0; // assuming acc == deceleration (m/s^2)

        // distances are in meters
        double total_distance = 10.0;
        double stopping_distance = 0.0;
        double lidar_distance = 0.1;     // from the front
        double safe_distance = 0.75;

        // buffer times are in seconds
        // sensing (1/sampling rate), communication, safety
        double buffer_time = (1.0 / 40) + 0.025 + 0.1;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_2d");
    ros::NodeHandle n;

    Chassis bot;

    bot.lidar_sub = n.subscribe("scan", 1000, &Chassis::obstacle_detection_callback, &bot);
    bot.state_sub = n.subscribe("SM_output", 1000, &Chassis::state_callback, &bot);
    bot.vehicle_state_sub = n.subscribe("/ground_truth/state", 1000, &Chassis::vehicle_state_callback, &bot);
    bot.hms_service = n.advertiseService("health_check_obstacle_2d", &Chassis::check, &bot);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}