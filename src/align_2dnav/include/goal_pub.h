#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>
#include "tf/tf.h"

#define NO_OF_SAMPLES_LASER			720
#define LEG_RADIUS					0.1
#define LENGTH_BIG_SIDE				2.8
#define ERROR_THRESHOLD_COMPARE		0.15

enum goal_pub_e
{
	GOAL_PUB_SUCCESS,
	GOAL_PUB_ERROR_LEG_COUNT_NOT_ENOUGH,
	GOAL_PUB_ERROR_TRANSFORM_EX,

};

class goal_publisher
{
	private:

		void laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan);
		goal_pub_e get_legs();
		goal_pub_e compute_goal_pose();

		ros::NodeHandle *node;
		ros::Subscriber laser_sub;
		ros::Publisher goal_pub;

		sensor_msgs::LaserScan laser_data;
		geometry_msgs::PoseStamped goal_pose;

		geometry_msgs::Point leg_points[4];
		tf::TransformListener listener;

		float_t get_table_pose_angle(geometry_msgs::Point point_1, geometry_msgs::Point point_2);
		void extrapolate_the_fourth_point(void);

	public:

		goal_pub_e get_goal();
		goal_pub_e publish_goal();
		goal_publisher(ros::NodeHandle* nodeH);
		~goal_publisher();
};
