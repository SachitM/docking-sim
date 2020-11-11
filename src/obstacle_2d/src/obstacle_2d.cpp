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

#define RANGE 1.22172999382         // angular range
#define INCREMENT 0.0043711271137   // angular increment between laser readings
#define LASER_STEPS 560

class Chassis {
    public:
        ros::Subscriber lidar_sub;
        ros::Subscriber state_sub;

        ros::Publisher obstacle_pub;
        ros::ServiceServer hms_service;

        Chassis() {
            // calculate maximum distance to detect obstacles
            double buffer_time = sense_buffer + comm_buffer + safe_buffer;
            double stopping_distance = (vel * vel * 0.5 / acc) + (buffer_time * vel);

            total_distance = stopping_distance + lidar_distance + safe_distance;

            update_ranges(width);
        }

        void obstacle_detection_callback(const sensor_msgs::LaserScan scan)
        {
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
            if (in_state->CurrState == state_machine::StateOut::State_D_Approach && is_approach == false)
            {
                double less_width = width / 2.0;
                update_ranges(less_width);
                ROS_INFO( "Beginning Approach - Constraining 2D obstacle detection width");
                is_approach = true;
            }
            else if (in_state->CurrState != state_machine::StateOut::State_D_Approach && is_approach == true)
            {
                update_ranges(width);
                ROS_INFO( "Using original 2D obstacle detection width.");
                is_approach = false;
            }
            else {
                return;
            }
        }

        void update_ranges(double new_width)
        {
            double angle = atan2(width, 2 * total_distance);

            double start = (RANGE - angle) / INCREMENT;
            int s = int(start);

            // need to add code for blind spot ditances
            for (int i = s; i < LASER_STEPS - s; i++) {
                range_array[i] = total_distance;
            }
        }


    private:
        bool hms_flag;
        bool is_approach = false;
        double range_array[LASER_STEPS] = {0};

        double width = 1.5;    // changed since 1.355 wasn not working correctly
        double vel = 3;
        double acc = 2; // assuming acc == deceleration

        // distances are in meters
        double total_distance = 3;      // to be overwritten in constructor
        double lidar_distance = 0.1;     // from the front
        double safe_distance = 0.3;

        // times are in seconds
        double sense_buffer = 1.0 / 40;    // since the lidar samples at 40 Hz
        double comm_buffer = 0.025;
        double safe_buffer = 0.1;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_2d");
    ros::NodeHandle n;

    Chassis bot;

    bot.lidar_sub = n.subscribe("scan", 1000, &Chassis::obstacle_detection_callback, &bot);
    bot.state_sub = n.subscribe("SM_output", 1000, &Chassis::state_callback, &bot);

    bot.hms_service = n.advertiseService("health_check_obstacle_2d", &Chassis::check, &bot);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
