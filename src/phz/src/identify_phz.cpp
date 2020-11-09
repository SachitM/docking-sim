#include <ros/ros.h>
#include "state_machine/StateOut.h"
#include "state_machine/StateIn.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <std_msgs/String.h>
#include <cmath>

#define RAD2DEG 57.295779513

ros::Publisher detection_pub;
std_msgs::String out_msg;

double target_x = 0.0;
double target_y = 0.0;

double min_dist = 2.0+0.5*1.9; //pose is of base_link at chassis rear

// only enable functionality of this node if set by enabled flag
bool EnableFlag = false;
bool CompletedP2P = false;


void phz_start_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(EnableFlag)
	{
		target_x =  msg->pose.position.x;
		target_y = msg->pose.position.y;
	}	
}


void ndt_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(EnableFlag)
	{
		double x = msg->pose.position.x;
		double y = msg->pose.position.y;

		double dist = sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y));

		if (dist<=min_dist){
			ROS_INFO("PHZ reached");
			ROS_INFO("Distance from PHZ entry point: %.2f m", dist);
			out_msg.data = "REACHED";
			detection_pub.publish(out_msg);

			// Update state feedback via publishing based on success flag and disable node flag
			CompletedP2P = true;
		} else {
			ROS_INFO("PHZ not reached");
		}
	}
}

void StateMachineCb(const state_machine::StateOut::ConstPtr& InStateInfo)
{
    ROS_INFO("CALLBACK TRIGGERED for p2p");
    // Enable goal pub if curr state is pod identification or approach
    if(InStateInfo -> CurrState == state_machine::StateOut::State_P2P)
    {
        EnableFlag = true;
        ROS_INFO("ENABLED flag for p2p");
    }
    else
    {
        EnableFlag = false;
    }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "identify_phz_node");
	ros::NodeHandle n;
	
	ros::Subscriber ndt_pose_sub = n.subscribe("ndt_pose", 1000, ndt_pose_CB);
	ros::Subscriber phz_start_sub = n.subscribe("phz_start_groundtruth", 1000, phz_start_CB);
	detection_pub = n.advertise<std_msgs::String>("detected", 1000);

	ros::Subscriber sm_sub = n.subscribe("SM_output", 10, StateMachineCb);
	ros::Publisher sm_pub = n.advertise<state_machine::StateIn>("SM_input", 10);

	ros::Rate loop_rate(10);

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		if(CompletedP2P)
		{
			state_machine::StateIn StateUpdate;
			// update the curr state under transition
			StateUpdate.TransState = state_machine::StateOut::State_P2P;
			// Whether or not it is completed
			StateUpdate.StateTransitionCond = 1;
			// Publish update
			sm_pub.publish(StateUpdate);
			CompletedP2P = false;
		}
	}

	return 0;

}