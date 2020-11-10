/*  
    author: Sachit Mahajan
*/
#include "april_tag_goal_pub.h"

aprilTagGoalPublisher::aprilTagGoalPublisher(ros::NodeHandle *nodeH) {
	this->node= nodeH;

	this->tag_sub = node->subscribe("/tag_detections", 1000, &aprilTagGoalPublisher::aprilTagDetectionCb, this);
    // ros::Subscriber sub = n.subscribe("/tag_detections", 1000, apriltag_detection_callback);
	this->tag_goal_pub = node->advertise<geometry_msgs::PoseStamped>("/pod_predicted_tag", 1);
    // State machine output is this nodes input (for determining current state)
    this -> state_sub = node -> subscribe("SM_output", 10, &aprilTagGoalPublisher::StateMachineCb, this);
    // State machine input is this nodes output (for determining transition condition)
    this -> state_pub = node -> advertise<state_machine::StateIn>("SM_input", 10);
	if (ros::param::has("/align/camera_offset")) {
		ros::param::get("/align/camera_offset", this->camera_offset);
	}
	else {
		this->camera_offset = OFFSET_CAMERA;
	}
}

void aprilTagGoalPublisher::aprilTagDetectionCb(const apriltag_ros::AprilTagDetectionArray msg) {
    if(!enable_goal_publishing) {
        return;
    }

    // if(FOUND_TWO_TAGS == tag_state_) {
    //     return;
    // }

    geometry_msgs::PoseStamped p;
    for(int i =0; i<msg.detections.size();i++) {
        p.pose = msg.detections[i].pose.pose.pose;
        p.header = msg.detections[i].pose.header;
        
        if(msg.detections[i].id[0] == target_tags.first) {
            // ROS_INFO("Found %d %d\n", msg.detections[i].id[0], tag_state_);
             
            if(FOUND_RIGHT_TAG == tag_state_ || FOUND_TWO_TAGS == tag_state_) {
                tag_state_ = FOUND_TWO_TAGS;
            }
            else {
                tag_state_ = FOUND_LEFT_TAG;
            }
            goal_pose_left = p;   
        }
        else if (msg.detections[i].id[0] == target_tags.second) {
            // ROS_INFO("Found %d %d\n", msg.detections[i].id[0], tag_state_);

            if(FOUND_LEFT_TAG == tag_state_ || FOUND_TWO_TAGS == tag_state_) {
                tag_state_ = FOUND_TWO_TAGS;
            }
            else {
                tag_state_ = FOUND_RIGHT_TAG;
            }
            goal_pose_right = p;    
        }
        else {
            /* Do Nothing */
        }
    } 
}

void aprilTagGoalPublisher::transformTagtoPodCenter(geometry_msgs::PoseStamped p, bool is_left) {
    int id = 0;
    geometry_msgs::PoseStamped goal_pose;
    bool valid_ = true;
    tf::Transform transform;
    tf::Transform transform_rot;
    transform_rot.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_rot.setRotation( tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2) );
    tf::Stamped<tf::Transform> tag_tf;
    tf::poseStampedMsgToTF(p, tag_tf);    
    tf::poseMsgToTF(p.pose, transform);
    transform = transform_rot * transform;
    tf::Transform transform_pod_center;
    if(is_left) {
        id = target_tags.first; 
        transform_pod_center.setOrigin( tf::Vector3(0.75031,-1.1256,-0.437018) );
    }
    else {
        id = target_tags.second;
        transform_pod_center.setOrigin( tf::Vector3(-0.72656,-1.12573,-0.437018) );
    }
    transform_pod_center.setRotation( tf::createQuaternionFromRPY(0,0,0) );
    transform = transform * transform_pod_center ;

    tag_tf.setOrigin(transform.getOrigin());
    tag_tf.setRotation(transform.getRotation());


    goal_pose.pose.position.x = tag_tf.getOrigin().x() + camera_offset;
    goal_pose.pose.position.y = tag_tf.getOrigin().y();
    goal_pose.pose.position.z = 0;

    float_t angle = M_PI/2 +  tf::getYaw(tag_tf.getRotation());
    goal_pose.header.frame_id = "/base_link";
    goal_pose.header.stamp = ros::Time(0);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angle), goal_pose.pose.orientation);

    if(is_left) {
        br.sendTransform(tf::StampedTransform(tag_tf, tag_tf.stamp_, "camera_link", "april_tf_left"));
        pose_yaws_.first = angle;
    }
    else {
        br.sendTransform(tf::StampedTransform(tag_tf, tag_tf.stamp_, "camera_link", "april_tf_right"));
        pose_yaws_.second = angle;
    }

    // Transform the Poses to World Frame
    try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        listener.transformPose("/map", goal_pose, goal_pose);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        valid_ = false;
    }

    if(valid_) {	
        tf::Pose pose_tf;	
        tf::poseMsgToTF(goal_pose.pose, pose_tf);
        tag_state_ = FOUND_TAG_TRANSFORM;
        ROS_INFO("[APRILTAG] ID %d, x %f y %f theta %f :", id, goal_pose.pose.position.x, goal_pose.pose.position.y, 
                                            tf::getYaw(pose_tf.getRotation()) * 180 /M_PI );
        if(is_left) {
            goal_pose_left = goal_pose;
        }
        else {
            goal_pose_right = goal_pose;
        }
        
    }
    else {
        ROS_ERROR("[APRILTAG] Unable to get a World to Base Transform");
    }

}

geometry_msgs::PoseStamped aprilTagGoalPublisher::averageCenters() {

    //TODO: Check how similar they are (Q: Use cov and mahalanobis?)
    // tfpose1.inverseTimes(tfpose2)

    geometry_msgs::PoseStamped pod_center;
    pod_center.pose.position.x = (goal_pose_left.pose.position.x + goal_pose_right.pose.position.x)/2;
    pod_center.pose.position.y = (goal_pose_left.pose.position.y + goal_pose_right.pose.position.y)/2;
    pod_center.pose.position.z = (goal_pose_left.pose.position.z + goal_pose_right.pose.position.z)/2;
    float_t pod_center_yaw = (pose_yaws_.first + pose_yaws_.second)/2;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pod_center_yaw), pod_center.pose.orientation);
    tag_state_ = FOUND_POD_CENTER;
    return pod_center;
    
}

void aprilTagGoalPublisher::publishGoal(geometry_msgs::PoseStamped pod_center) {
    state_machine::StateIn StateUpdate;
    // update the curr state under transition
    StateUpdate.TransState = state_machine::StateOut::State_Identify;
    // Whether or not it is completed
    StateUpdate.StateTransitionCond = (isIdentifyPod) ? 1 : 0;
    if(isIdentifyPod)
    {
        state_pub.publish(StateUpdate);
        ROS_INFO("PUBLISHED");
    }
    tag_goal_pub.publish(pod_center);
}

void aprilTagGoalPublisher::StateMachineCb(const state_machine::StateOut::ConstPtr& InStateInfo)
{
    ROS_INFO("CALLBACK TRIGGERED");
    // Assume only the last two digits to be valid
    target_tags = {InStateInfo -> PodInfo / 10, InStateInfo -> PodInfo % 10};
    // Enable goal pub if curr state is pod identification
    if(InStateInfo -> CurrState == state_machine::StateOut::State_Identify || InStateInfo -> CurrState == state_machine::StateOut::State_D_Approach)
    {
        enable_goal_publishing = true;
        ROS_INFO("ENABLED");
    }
    else
    {
        enable_goal_publishing = false;
    }
    // enable_goal_publishing = (InStateInfo -> CurrState == state_machine::StateOut::State_Identify) ? true : false;
    isIdentifyPod = (InStateInfo -> CurrState == state_machine::StateOut::State_Identify) ? true : false;
}
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "pod_localizer");
    ros::NodeHandle n;
    aprilTagGoalPublisher agp(&n);
    ros::Rate loop_rate(5);
    ros::Duration(1).sleep();
    ROS_INFO("aprilTag Localizer has started");

    //TODO: Change this to STATE MACHINE
    // agp.target_tags = {1,2};
    while (ros::ok())
	{
        geometry_msgs::PoseStamped pod_center;
		agp.tag_state_ = FOUND_NO_TAGS;

		ros::spinOnce();
        //TODO: enable with sm
        if(agp.enable_goal_publishing) {
            switch(agp.tag_state_) {
                case FOUND_LEFT_TAG:
                    agp.transformTagtoPodCenter(agp.goal_pose_left, true);
                    if(FOUND_TAG_TRANSFORM == agp.tag_state_) {
                        agp.tag_state_ = FOUND_POD_CENTER;
                        pod_center = agp.goal_pose_left;
                    }
                break;
                case FOUND_RIGHT_TAG:
                    agp.transformTagtoPodCenter(agp.goal_pose_right, false);
                    if(FOUND_TAG_TRANSFORM == agp.tag_state_) {
                        agp.tag_state_ = FOUND_POD_CENTER;
                        pod_center = agp.goal_pose_right;
                    }
                break;
                case FOUND_TWO_TAGS:
                    agp.transformTagtoPodCenter(agp.goal_pose_left, true);
                    agp.transformTagtoPodCenter(agp.goal_pose_right, false);
                    pod_center = agp.averageCenters();
                break;
            }

            if(FOUND_POD_CENTER == agp.tag_state_){
                 agp.publishGoal(pod_center);
            }
            else {
                
                 ROS_ERROR("[APRILTAG] Unable to Localize POD %d", agp.tag_state_ );
            }

        }
		loop_rate.sleep();
	}
    return 0;
}
