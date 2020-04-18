#!/usr/bin/env python

import numpy as np
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

x_lim = [-6, 1]
y_lim = [-2, 2]
theta_lim = [-np.pi, np.pi]
num_waypoints = 10
waypoint_tol = 0.05

wheelbase = 1.9
max_acc = 3
max_steering_angle = 0.7
