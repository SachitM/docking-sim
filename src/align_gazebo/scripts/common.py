#!/usr/bin/env python

import numpy as np
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

x_lim = [-4, 1]
y_lim = [-2, 2]
theta_lim = [-np.pi, np.pi]
num_waypoints = 5
waypoint_tol = [0.15, 0.07,0.05,0.04,0.04]

retrace_waypoint_tol = 0.1

wheelbase = 1.9
max_acc = [2, 1, 1, 1, 0.7, 0.3]
max_steering_angle = [0.62,0.62,0.62,0.3,0.1]
max_steering_angle_ret = 0.2
MAX_VEL = 3
Kp = 0.9
Kd = 0.4
