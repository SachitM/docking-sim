#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from tf.transformations import quaternion_from_euler

from common import *

if __name__ == '__main__':

  rospy.init_node('waypoint_publisher')

  waypoints = np.zeros((num_waypoints, 3))

  waypoints[0,0] = 33.0
  waypoints[0,1] = 33.5
  waypoints[1,0] = 33.0
  waypoints[1,1] = 34.0
  waypoints[2,0] = 33.0
  waypoints[2,1] = 35.0
  waypoints[3,0] = 33.0
  waypoints[3,1] = 36.0
  waypoints[4,0] = 33.0
  waypoints[4,1] = 37.0

  # Waypoint publisher
  waypoints_pub = rospy.Publisher('/waypoints_goal', PoseArray, queue_size=10)

  waypoint_array = PoseArray()
  for i in range(0, num_waypoints):
    w = Pose()
    w.position.x = waypoints[i, 0]
    w.position.y = waypoints[i, 1]
    w.orientation = Quaternion(*quaternion_from_euler(0, 0, waypoints[i, 2]))

    waypoint_array.poses.append(w)  
  # Publish the waypoints
  r = rospy.Rate(1)  # 10hz
  while not rospy.is_shutdown():
    waypoints_pub.publish(waypoint_array)
    waypoints[1:5,0] += np.random.uniform(-0.2,0.2,1)
    # print(waypoint_array)
    r.sleep()



