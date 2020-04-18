#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

import tf

# New frame called odom to provide static link to /map and dynamic link to /base_link
# Mainly to synchronize rviz and gazebo
def broadcastTF(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "base_link",
                     "odom")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    rospy.Subscriber("/align/ground_truth/state",
                     Odometry,
                     broadcastTF)
    rospy.spin()
