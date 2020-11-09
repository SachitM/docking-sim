/*  
	authors: Poorva Agrawal, Uma Arunachalam
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "state_machine/StateIn.h"
#include "state_machine/StateOut.h"
#include <sstream>
#include <string>
#include <map>


class StateMachineNode{
	public:
		StateMachineNode(ros::NodeHandle *nh);
		~StateMachineNode(){
		}
		void OpPublisher();
		
	private:
		uint prev_state = 0;
		uint curr_state = 0;
		uint hms_check  = 0;
		uint op_mode    = 0;
		uint info       = 0;
		int curr_state_update = -1;
		bool hms_active = false;
		bool isPod      = false;

		ros::NodeHandle* node;
		ros::Publisher output_pub;
		ros::Subscriber input_sub;
		ros::Subscriber hms_sub;
		state_machine::StateOut out_msg;

		std::map<uint, std::string> console_m;
		std::map<uint, std::string> console_s;
		
		void ConsoleOut(std::string action);
		void HMSCallback(const std_msgs::String::ConstPtr& msg);        
		void IpCallback(const state_machine::StateIn::ConstPtr& msg);
		void StateTransition(const state_machine::StateIn::ConstPtr& msg);
};

/**
 * StateMachineNode class constructor 
 */
StateMachineNode::StateMachineNode(ros::NodeHandle *nh){

	// initialise node vars
	node       = nh;
	prev_state = state_machine::StateOut::State_Idle;
	curr_state = state_machine::StateOut::State_Idle;
	op_mode    = state_machine::StateOut::OperationMode_Standby;
	hms_check  = 0;

	
	// initialise pubs and subs
	output_pub = node->advertise<state_machine::StateOut>("SM_output", 1000);
	input_sub  = node->subscribe("SM_input", 10, &StateMachineNode::IpCallback, this);
	hms_sub    = node->subscribe("HMS_Status", 1, &StateMachineNode::HMSCallback, this);
	
	// initialise out msg
	out_msg.HMSCheck      = hms_check;
	out_msg.OperationMode = op_mode;
	out_msg.PrevState     = prev_state;
	out_msg.CurrState     = curr_state;
	// @todo: set this value via pod server
	out_msg.PodInfo       = info;

	// Add console output messages to mode dict
	console_m[state_machine::StateOut::OperationMode_Standby] = "Standby";
	console_m[state_machine::StateOut::OperationMode_Pickup] = "Pick Up";
	console_m[state_machine::StateOut::OperationMode_DropOff] = "Drop Off";

	// Add console output messages to state dict
	console_s[state_machine::StateOut::State_Idle] = "Idle";
	console_s[state_machine::StateOut::State_P2P] = "P2P";
	console_s[state_machine::StateOut::State_Identify] = "Identify";
	console_s[state_machine::StateOut::State_Approach] = "Approach Navigation";
	console_s[state_machine::StateOut::State_Verify] = "Verify Pose";
	console_s[state_machine::StateOut::State_Retrace] = "Retrace";
	console_s[state_machine::StateOut::State_Lock] = "Dock with Pod";
	console_s[state_machine::StateOut::State_Unlock] = "Undock with Pod";
	console_s[state_machine::StateOut::State_EHS] = "Emergency Handling State";

	ROS_INFO("[SM] Chassis Initialised in STANDBY MODE");
	ROS_INFO("[SM] Waiting for HMS");		
}


/**
 *  Publish current state information 
 */
void StateMachineNode::OpPublisher(){
	
	if (hms_active){
		out_msg.HMSCheck      = hms_check;
		out_msg.OperationMode = op_mode;
		out_msg.PrevState     = prev_state;
		out_msg.CurrState     = curr_state;
		out_msg.PodInfo       = info;
		output_pub.publish(out_msg);
	}
	return;
}

/**
 * Print to console
 */
void StateMachineNode::ConsoleOut(std::string action){
	ROS_INFO("[SM] MODE: %s, CURRENT STATE: %s, LAST STATE: %s, UPDATE: %s", console_m[op_mode].c_str(), console_s[curr_state].c_str(), console_s[prev_state].c_str(), action.c_str());				
	return;
}

/**
 * HMS_Status topic callback
 */
void StateMachineNode::HMSrostopic pub /SM_input state_machine/StateIn '{State: 0, StateUpdate: 12, OperationMode: 1}'Callback(const std_msgs::String::ConstPtr& msg){
	// Initialisation check: state machine should operate only if HMS is activated

	std::string action = "-";

	if (!hms_active){
		ROS_INFO("[SM] HMS Active");
		hms_active = true;
		ROS_INFO("[SM] Chassis Ready For Operation");
		action = "Starting Up";
		ConsoleOut(action);
	}	

	// update hms_check variable and state
	hms_check = (msg->data == "Passed") ? 1 : 0;
	
	if ((state_machine::StateOut::State_EHS != curr_state) && (0 == hms_check)){
		prev_state = curr_state;
		curr_state = state_machine::StateOut::State_EHS;
		action = "System Failure Detected";
		ConsoleOut(action);
	} 
	if ((state_machine::StateOut::State_EHS == curr_state)&&(1 == hms_check)){
		curr_state = prev_state;
		prev_state = state_machine::StateOut::State_EHS;
		std::string action = "Diagnostics Complete";
		ConsoleOut(action);
	}
	return;
}

/**
 * SM input callback
 */
void StateMachineNode::IpCallback(const state_machine::StateIn::ConstPtr& msg){
	// update curr_state_update when an update is received from a node
	if (hms_active){
		if (msg->TransState==curr_state){
			curr_state_update = msg->StateTransitionCond;
			StateTransition(msg);
		}
	}
	return;
}

/**
 * SM transitions implementation
 */
void StateMachineNode::StateTransition(const state_machine::StateIn::ConstPtr& msg){
	// Todo : handle unsuccessful cases
	// ROS_INFO("%d", curr_state_update);
	if ((hms_active)&&(curr_state_update!=-1)){
		
		std::string action;
		// If in IDLE state and STANDBY mode
		// assign mode and info
		if ((op_mode == state_machine::StateOut::OperationMode_Standby)&&(curr_state == state_machine::StateOut::State_Idle)){
			// Todo : Check if pod is present or not for the req mode?
			op_mode = msg->OperationMode;
			info    = msg->StateTransitionCond;
			prev_state = curr_state;
			curr_state = state_machine::StateOut::State_P2P;

			action  = "Input received";
			ConsoleOut(action);

		} else if (state_machine::StateOut::OperationMode_Pickup == op_mode){

			
			switch(curr_state){

				case state_machine::StateOut::State_Idle :
					action  = "No action taken";
					ConsoleOut(action);
					break;

				case state_machine::StateOut::State_P2P :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Identify;
						action  = "Destination Reached";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Identify :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Approach;
						action  = "PHZ Correctly Identified";
						ConsoleOut(action);
					} else if (0 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_P2P;
						action  = "Incorrect PHZ reached";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Approach :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Verify;	
						action  = "Approach Complete";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Verify :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Lock;
						action  = "Docking Pose Achieved";
						ConsoleOut(action);
					} else if (0 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Retrace;
						action  = "Error in Pose";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Retrace :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Lock;
						action  = "Retrace Complete";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Lock :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Idle;
						op_mode    = state_machine::StateOut::OperationMode_Standby;
						isPod      = true;
						action     = "Locking Successful";
						ConsoleOut(action);
					}
					break;	

				default :
					break;
			}
		} else if (state_machine::StateOut::OperationMode_DropOff == op_mode){

			switch(curr_state){

				case state_machine::StateOut::State_Idle :
					action  = "No action taken";
					ConsoleOut(action);
					break;
					
				case state_machine::StateOut::State_P2P :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Unlock;
						action  = "Destination Reached";
						ConsoleOut(action);
					}
					break;

				case state_machine::StateOut::State_Unlock :
					if (1 == curr_state_update){
						prev_state = curr_state; 
						curr_state = state_machine::StateOut::State_Idle;
						op_mode    = state_machine::StateOut::OperationMode_Standby;
						isPod      = false;
						action  = "Unlocking Successful";
						ConsoleOut(action);
					}
					break;	

				default :
					break;
			}

		}
		
		
		curr_state_update = -1;
	}

	return;			 
}

int main(int argc, char **argv){

	ros::init(argc, argv, "state_machine");
	ros::NodeHandle nh;
	
	StateMachineNode sm_node = StateMachineNode(&nh);

	ros::Rate loop_rate(10);

	while (ros::ok()){
		sm_node.OpPublisher();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
