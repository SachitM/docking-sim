#!/usr/bin/env python

# Author: Sachit Mahajan

import rospy
from nav_msgs.msg import Odometry

import tf
enable_second_odom = False
# New frame called odom to provide static link to /map and dynamic link to /base_link
# Mainly to synchronize rviz and gazebo
def broadcastTF(msg):

    if enable_second_odom == False:
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                        (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                        msg.header.stamp,
                        "base_link",
                        "odom")

    if enable_second_odom:
        new_msg = Odometry()
        new_msg = msg
        new_msg.header.frame_id = "base_link"
        new_msg.child_frame_id = "odom"
        pub_.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    pub_ = rospy.Publisher('/odom_pose', Odometry)

    rospy.Subscriber("/ground_truth/state",
                     Odometry,
                     broadcastTF)
    rospy.spin()
