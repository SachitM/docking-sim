#include <ros/ros.h>
#include <apriltag.h>
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "state_machine/StateOut.h"
#include "state_machine/StateIn.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include<vector>
#define OFFSET_CAMERA 1.1 //+0.95
// TODO: Move to param server

enum tag_state_e {
    FOUND_NO_TAGS = 0,
    FOUND_TWO_TAGS,
    FOUND_LEFT_TAG,
    FOUND_RIGHT_TAG,
    FOUND_TAG_TRANSFORM,
    FOUND_POD_CENTER,

};

class aprilTagGoalPublisher {
    private:

        ros::NodeHandle *node;
		ros::Subscriber tag_sub;
        ros::Subscriber state_sub;
		ros::Publisher tag_goal_pub;
        ros::Publisher state_pub;
        tf::TransformListener listener;
        tf::TransformBroadcaster br;
        void aprilTagDetectionCb(const apriltag_ros::AprilTagDetectionArray msg);
        void StateMachineCb(const state_machine::StateOut::ConstPtr& StateInfo);

        float_t camera_offset;
        std::pair<float_t,float_t> pose_yaws_;
        
    public:
        // aprilTagGoalPublisher() {}
        aprilTagGoalPublisher(ros::NodeHandle* nodeH);
        ~aprilTagGoalPublisher(){
        }

        void transformTagtoPodCenter(geometry_msgs::PoseStamped p, bool is_left);
        void identifyPod();
        geometry_msgs::PoseStamped averageCenters();
        void publishGoal(geometry_msgs::PoseStamped pod_center);
        
        std::pair<int,int> target_tags;
        
        bool enable_goal_publishing = false;//true;
        bool isIdentifyPod = false;

        tag_state_e tag_state_ = FOUND_NO_TAGS;
        geometry_msgs::PoseStamped goal_pose_left;
        // tf::Stamped<tf::Transform> tag_tf_left;
        geometry_msgs::PoseStamped goal_pose_right;
        // tf::Stamped<tf::Transform> tag_tf_right;
};
