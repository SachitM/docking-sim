#include "ros/ros.h"
#include "std_msgs/String.h"
#include "state_machine/StateOut.h"
#include "state_machine/StateIn.h"
#include <sstream>

// @todo: remove global var, use class methods
int SourceState = 0;
int DestState = 1;
int HMS_check = 1;

void updateCallback(const state_machine::StateIn::ConstPtr& msg)
{
    // if (msg->PodIdentification == 1){
    //     ROS_INFO("[MODE: %s, HMS CHECK: %s, UPDATE: %s, STATE: %s ]\n", "Pod PickUp", "Passed", "ARTag Detected","Pod Identification" );
    // } else if (msg->Approach == 1){
    //     ROS_INFO("[MODE: %s, HMS CHECK: %s, UPDATE: %s, STATE: %s ]\n", "Pod PickUp", "Passed", "Pod Identified", "Approach");
    // } else if (msg->Verification == 1){
    //     ROS_INFO("[MODE: %s, HMS CHECK: %s, UPDATE: %s, STATE: %s ]\n", "Pod PickUp", "Passed", "Approach Complete", "Verify Pose");
    // } else if (msg->Docking == 1){
    //     ROS_INFO("[MODE: %s, HMS CHECK: %s, UPDATE: %s, STATE: %s ]\n", "Pod PickUp", "Passed", "Pose Achieved", "Lock");
    // }
    if(msg -> Transitions == 1)
    {
        SourceState = 1;
        DestState = 2;
    }
}

void HMSStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    HMS_check = (msg->data.c_str() == "Passed") ? 1 : 0;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "StateMachineMsgPublisher");
    ros::NodeHandle StateMachineNode;
    ros::Publisher sm_pub = StateMachineNode.advertise<state_machine::StateOut>("system_status", 1000);
    ros::Subscriber sm_sub = StateMachineNode.subscribe("state_update", 1000, updateCallback);
    ros::Subscriber hms_sub = StateMachineNode.subscribe("HMS_Status", 1, HMSStatusCallback);
    // ROS_INFO("[MODE: %s, HMS CHECK: %s, STATE: %s \n]", "Pod PickUp", "Passed", "Idle" );

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        state_machine::StateOut OutMsg;
        OutMsg.HMSCheck = HMS_check;
        OutMsg.OperationMode = 1;
        OutMsg.SourceState = SourceState;
        OutMsg.DestState = DestState;
        sm_pub.publish(OutMsg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
