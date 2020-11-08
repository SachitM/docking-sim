/*  
    authors: Poorva Agrawal, Uma Arunachalam
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "state_machine/StateIn.h"
#include "state_machine/StateOut.h"
#include <sstream>


class StateMachineNode{
    public:
        StateMachineNode(ros::NodeHandle *nh);
        void OpPublisher();
        ros::Publisher output_pub;
        ros::Subscriber input_sub;
        ros::Subscriber hms_sub;
    private:
        uint prev_state = 0;
        uint curr_state = 1;
        uint hms_check  = 0;
        uint op_mode    = 0;
        ros::NodeHandle* node;
        state_machine::StateOut out_msg;
        void HMSCallback(const std_msgs::String::ConstPtr& msg);        
        void IpCallback(const state_machine::StateIn::ConstPtr& msg);
};

StateMachineNode::StateMachineNode(ros::NodeHandle *nh){

    // initialise node vars
    node       = nh;
    prev_state = state_machine::StateOut::State_Idle;
    curr_state = state_machine::StateOut::State_P2P;
    op_mode    = state_machine::StateOut::OperationMode_Pickup;
    hms_check  = 1;
    
    // initialise pubs and subs
    output_pub = node->advertise<state_machine::StateOut>("SM_output", 1000);
    input_sub  = node->subscribe("SM_input", 1000, &StateMachineNode::IpCallback, this);
    hms_sub    = node->subscribe("HMS_Status", 1, &StateMachineNode::HMSCallback, this);
    
    // initialise out msg
    out_msg.HMSCheck      = hms_check;
    out_msg.OperationMode = op_mode;
    out_msg.PrevState     = prev_state;
    out_msg.CurrState     = curr_state;
    out_msg.PodInfo       = 12;
}

void StateMachineNode::HMSCallback(const std_msgs::String::ConstPtr& msg){
    // update hms_check variable and state
    hms_check = (msg->data.c_str() == "Passed") ? 1 : 0;
    if (0 == hms_check){
        curr_state = state_machine::StateOut::State_EHS;
    }   
}

void StateMachineNode::IpCallback(const state_machine::StateIn::ConstPtr& msg){
//     //     if(msg -> Transitions == 1)
//     // {
//     //     SourceState = 1;
//     //     DestState = 2;
//     // }
}

void StateMachineNode::OpPublisher(){
    out_msg.HMSCheck      = hms_check;
    out_msg.OperationMode = op_mode;
    out_msg.PrevState     = prev_state;
    out_msg.CurrState     = curr_state;
    output_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle nh;
    StateMachineNode sm_node = StateMachineNode(&nh);
    
    // ROS_INFO("[MODE: %s, HMS CHECK: %s, STATE: %s \n]", "Pod PickUp", "Passed", "Idle" );

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sm_node.OpPublisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
