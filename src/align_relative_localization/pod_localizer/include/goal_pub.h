#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "state_machine/StateOut.h"
#include "tf/tf.h"

#define NO_OF_SAMPLES_LASER			896
// #define NO_OF_SAMPLES_LASER			726
#define LEG_RADIUS					0.02
#define LENGTH_BIG_SIDE				1.74
#define LENGTH_SHORT_SIDE			0.9
#define ERROR_THRESHOLD_COMPARE		0.05
#define MAX_RANGE_ALLOWED			4
#define MIN_RANGE_ALLOWED			0.3
#define SAMPLES_SKIPPED				100 //40 Corresponds to 10degrees
#define CIRCLE_RADIUS				1.4

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
		int runn = 0;
		void laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan);
		void prior_cb(const geometry_msgs::PoseStamped& pose_msg);
		bool inCircle(float_t x, float_t y);
		goal_pub_e get_legs();
		goal_pub_e compute_goal_pose();

		ros::NodeHandle *node;
		ros::Subscriber laser_sub;
		ros::Subscriber prior_sub;
		ros::Publisher goal_pub;
		ros::Publisher detected_legs_pub; 
		ros::Subscriber state_sub;

		sensor_msgs::LaserScan laser_data;
		geometry_msgs::PoseStamped goal_pose;

		geometry_msgs::Point leg_points[4];
		tf::TransformListener listener;
		tf::TransformBroadcaster br;
		
		float_t get_table_pose_angle(geometry_msgs::Point point_1, geometry_msgs::Point point_2);
		void extrapolate_the_fourth_point(void);
		void StateMachineCb(const state_machine::StateOut::ConstPtr& StateInfo);
		float_t lidar_offset;

		void publish_tf();

		bool prior_set = false;
		bool transformed_prior = false;
		geometry_msgs::PoseStamped prior_pose;
		std::pair<float_t,float_t> pod_prior_lidar_frame;

	public:
		bool EnableGoalPub = true;
		bool isval = false;
		goal_pub_e get_goal();
		goal_pub_e publish_goal();
		goal_publisher(ros::NodeHandle* nodeH);
		~goal_publisher();
};
