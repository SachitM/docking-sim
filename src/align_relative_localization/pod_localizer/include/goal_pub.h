#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>
#include "tf/tf.h"

#define NO_OF_SAMPLES_LASER			560
#define LEG_RADIUS					0.02
#define LENGTH_BIG_SIDE				1.76
#define LENGTH_SHORT_SIDE			0.9
#define ERROR_THRESHOLD_COMPARE		0.05
#define MAX_RANGE_ALLOWED			8
#define SAMPLES_SKIPPED				0 //40 Corresponds to 10degrees
#define CIRCLE_RADIUS				1.5

//TODO Move to param server
#define BASE_LINK_OFFSET_X			1.1 //+0.95

enum goal_pub_e
{
	GOAL_PUB_SUCCESS,
	GOAL_PUB_ERROR_LEG_COUNT_NOT_ENOUGH,
	GOAL_PUB_ERROR_TRANSFORM_EX,
	GOAL_TWO_LEG_ESTIMATE,

};

class goal_publisher
{
	private:

		void laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan);
		void prior_cb(const geometry_msgs::PoseStamped& pose_msg);
		bool inCircle(float_t x, float_t y);
		goal_pub_e get_legs();
		goal_pub_e compute_goal_pose();

		ros::NodeHandle *node;
		ros::Subscriber laser_sub;
		ros::Subscriber prior_sub;
		ros::Publisher goal_pub;

		sensor_msgs::LaserScan laser_data;
		geometry_msgs::PoseStamped goal_pose;

		geometry_msgs::Point leg_points[4];
		tf::TransformListener listener;

		float_t get_table_pose_angle(geometry_msgs::Point point_1, geometry_msgs::Point point_2);
		void extrapolate_the_fourth_point(void);

		float_t lidar_offset;
		bool prior_set = false;
		bool transformed_prior = false;
		geometry_msgs::PoseStamped prior_pose;
		std::pair<float_t,float_t> pod_prior_lidar_frame;
	public:

		goal_pub_e get_goal();
		goal_pub_e publish_goal();
		goal_publisher(ros::NodeHandle* nodeH);
		~goal_publisher();
};
