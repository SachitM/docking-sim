
import numpy as np
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *

num_waypoints = 5
waypoint_tol = 0.1
retrace_waypoint_tol = 0.15

wheelbase = 1.9
ODOM_INF = "/ground_truth/state"