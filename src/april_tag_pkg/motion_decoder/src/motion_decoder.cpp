#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>

ImageConverter* ic;

// AprilTagDetection
// int32 id
// float64 size
// geometry_msgs/PoseStamped pose
//   std_msgs/Header header
//     uint32 seq
//     time stamp
//     string frame_id
//   geometry_msgs/Pose pose
//     geometry_msgs/Point position
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Quaternion orientation
//       float64 x
//       float64 y
//       float64 z
//       float64 w


//TODO Move to param server
#define OFFSET_CAMERA 1.1 //+0.95

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  
  geometry_msgs::PoseStamped goal_pose_1;
  geometry_msgs::PoseStamped goal_pose_2;
  tf::TransformListener listener;
  //TODO: Parse message and publish transforms as apriltag_tf and camera
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform_rot;
  transform_rot.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform_rot.setRotation( tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2) );
  tf::Pose pose_tf;

  

  geometry_msgs::PoseStamped p;

  //Size will always be 1 for the rosbag
  for(int i =0; i<msg.detections.size();i++)
  {
   ROS_INFO("Found %d \n", msg.detections[i].id);

    // ROS_INFO_STREAM(msg.detections[i]);
     p.pose = msg.detections[i].pose.pose;
     p.header = msg.detections[i].pose.header;
     tf::Stamped<tf::Transform> tag_tf;
    
     tf::poseStampedMsgToTF(p, tag_tf);    
     tf::poseMsgToTF(p.pose, transform);
     transform = transform_rot * transform;

    tf::Transform transform_pod_center;
    // transform_pod_center.setOrigin( tf::Vector3(0.75031, 0.437018, 1.1256) );
    
    if(msg.detections[i].id == 1)
    {
      // transform_pod_center.setOrigin( tf::Vector3(0.75031,-1.1256,-0.437018) );
      transform_pod_center.setOrigin( tf::Vector3(0.75031,-1.1256,-0.437018) );
      // goal_pose_1.pose = T
    }
    else
    {
      transform_pod_center.setOrigin( tf::Vector3(-0.72656,-1.12573,-0.437018) );
      
    }
    transform_pod_center.setRotation( tf::createQuaternionFromRPY(0,0,0) );
    transform = transform * transform_pod_center ;

    // transform_rot.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // transform_rot.setRotation( tf::createQuaternionFromRPY(0,0,0) );

    // transform = transform * transform_rot ;

     tag_tf.setOrigin(transform.getOrigin());
     tag_tf.setRotation(transform.getRotation());
     
     if(2 == msg.detections.size() )
     {

       bool valid_ = true;
      // tf::transformTFToMsg(tag_tf, goal_pose_1);
      // auto &hol = goal_pose_1
       
       if(msg.detections[i].id == 1)
       { 
          br.sendTransform(tf::StampedTransform(tag_tf, tag_tf.stamp_, "camera_link", "april_tf_1"));
          goal_pose_1.pose.position.x = tag_tf.getOrigin().x() + OFFSET_CAMERA;
          goal_pose_1.pose.position.y = tag_tf.getOrigin().y();
          goal_pose_1.pose.position.z = 0;


          float_t angle = M_PI/2 +  tf::getYaw(tag_tf.getRotation());
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angle), goal_pose_1.pose.orientation);
          goal_pose_1.header.frame_id = "/base_link";
		      goal_pose_1.header.stamp = ros::Time(0);
          try
          {
            listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
            listener.transformPose("/map", goal_pose_1, goal_pose_1);
          }

          catch (tf::TransformException &ex)
          {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            valid_ = false;
          }

          if(valid_)
          {		
		        tf::poseMsgToTF(goal_pose_1.pose, pose_tf);

            ROS_INFO("ID %d, x %f y %f theta %f :", msg.detections[i].id, goal_pose_1.pose.position.x, goal_pose_1.pose.position.y, 
                                                    tf::getYaw(pose_tf.getRotation()) * 180 /M_PI );
          }
       }
      else
      {   
        br.sendTransform(tf::StampedTransform(tag_tf, tag_tf.stamp_, "camera_link", "april_tf_2"));
        goal_pose_2.pose.position.x = tag_tf.getOrigin().x() + OFFSET_CAMERA;
        goal_pose_2.pose.position.y = tag_tf.getOrigin().y();
        goal_pose_2.pose.position.z = 0;
        float_t angle = M_PI/2 +  tf::getYaw(tag_tf.getRotation());
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angle), goal_pose_2.pose.orientation);
        goal_pose_2.header.frame_id = "/base_link";
        goal_pose_2.header.stamp = ros::Time(0);

        try
        {
          listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
          listener.transformPose("/map", goal_pose_2, goal_pose_2);
        }

        catch (tf::TransformException &ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          valid_ = false;
        }

        if(valid_)
        {
          tf::poseMsgToTF(goal_pose_2.pose, pose_tf);

          ROS_INFO("ID %d, x %f y %f theta %f :", msg.detections[i].id, goal_pose_2.pose.position.x, goal_pose_2.pose.position.y, 
                                                  tf::getYaw(pose_tf.getRotation()) * 180 /M_PI );
        }
      }

      // ROS_INFO("ID %d, x %f y %f theta %f :", msg.detections[i].id, tag_tf.getOrigin().x(),tag_tf.getOrigin().y(), (M_PI/2 + tf::getYaw(tag_tf.getRotation()))* 180 /M_PI );
      // p.pose.position = tag_tf.getOrigin();
      // p.pose.orientation = tag_tf.getRotation();

    }
    //  Test If points are correct
     ic->setTagLocations(p.pose.position.x,p.pose.position.y, p.pose.position.z, msg.detections[i].id-1);
  } 

   ic->setTagLocations(-1,-1,-1,-1);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle n;
  //TODO: Add a subscriber to get the AprilTag detections The callback function skelton is given.

  ros::Subscriber sub = n.subscribe("/tag_detections", 1000, apriltag_detection_callback);

  ImageConverter converter;
  ic = &converter;
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
