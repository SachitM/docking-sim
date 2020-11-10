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
            float min_las = 2500;
            // iterating over all values since ideally all element in range should be non zero
            for (int i = 0; i < LASER_STEPS; i++) {
                min_las = std::min(min_las, scan.ranges[i]);
                if (range_array[i] > scan.ranges[i]) {
                    if (!obstacle_flag) {
                       ROS_INFO( "Obstacle detected!");
                    }
                    hms_flag = true;
                    return;
                }
            }
            ROS_INFO( "Nearest Entity : %f", min_las);

            obstacle_flag = false;
        }

        bool check(hms_client::ping_pong::Request  &req,
                 hms_client::ping_pong::Response &res)
        {
            res.msg.header.stamp = ros::Time::now();
            res.health = 1;
            res.error_code = hms_flag ? 1 : 0;
            // std::cout << "Error code! " << res.error_code << std::endl;
            hms_flag = false;
            return true;
        }

        void state_callback(const state_machine::StateOut::ConstPtr& in_state)
        {
            if (in_state->CurrState == state_machine::StateOut::State_D_Approach && is_approach == false)
            {
                double less_width = width / 2.0;
                update_ranges(less_width);
                is_approach = true;
            }
            else if (in_state->CurrState != state_machine::StateOut::State_D_Approach && is_approach == true)
            {
                update_ranges(width);
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
        bool obstacle_flag = false;
        bool is_approach = false;
        double range_array[LASER_STEPS] = {0};

        double width = 1.85;    // changed since 1.355 wasn not working correctly
        double vel = 2;
        double acc = 3; // assuming acc == deceleration

        // distances are in meters
        double total_distance = 5;      // to be overwritten in constructor
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
