#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <hms_client/hms_msg.h>
#include <hms_client/ping_pong.h>
#include <math.h>

#define RANGE 1.22172999382
#define INCREMENT 0.0043711271137
#define STEPS 560

class Chassis{
    public:
        ros::Subscriber lidar_sub;
        ros::Publisher obstacle_pub;
        ros::ServiceServer hms_service;

        Chassis(){
            // calculate maximum distance to detect obstacles
            double buffer_time = sense_buffer + comm_buffer + safe_buffer;
            double stopping_distance = (vel * vel * 0.5 / acc) + (buffer_time * vel);

            total_distance = stopping_distance + lidar_distance + safe_distance;
            std::cout << "Calculated distance: " << total_distance << std::endl;

            double angle = atan2(width, 2 * total_distance);
            std::cout << "Calculated angle: " << angle * 180 / 3.14 << std::endl;

            double start = (RANGE - angle) / INCREMENT;
            int s = int(start);

            // need to add code for blind spot ditances
            for (int i = s; i < STEPS - s; i++){
                range_array[i] = total_distance;
            }
        }

        void obstacle_detection_callback(const sensor_msgs::LaserScan scan)
        {
            // iterating over all values since ideally all element in range should be non zero
            for (int i = 0; i < STEPS; i++){
                if (range_array[i] > scan.ranges[i]){
                    // std::cout << scan.ranges[i] << std::endl;
                    if (!obstacle_flag){
                        std::cout << "Obstacle detected!\n";
                    }

                    obstacle_flag = true;
                    hms_flag = true;
                    msg.linear.x = 0;
                    obstacle_pub.publish(msg);
                    return;
                }
            }
            
            obstacle_flag = false;

        }

        bool check(hms_client::ping_pong::Request  &req,
                 hms_client::ping_pong::Response &res)
        {
            res.msg.header.stamp = ros::Time::now();
            res.health = 1;

            if(hms_flag)
                res.error_code = 1;
            else
                res.error_code = 0;

            std::cout << "Error code! " << res.error_code << std::endl;
            hms_flag = false;
            return true;
        }


    private:
        bool obstacle_flag;
        bool hms_flag;
        double range_array[STEPS] = {0};
        geometry_msgs::Twist msg;

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
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_2d");
    ros::NodeHandle n;

    Chassis bot;

    bot.lidar_sub = n.subscribe("scan", 1000, &Chassis::obstacle_detection_callback, &bot);
    bot.obstacle_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    bot.hms_service = n.advertiseService("health_check_obstacle_2d", &Chassis::check, &bot);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
