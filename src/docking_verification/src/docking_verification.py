#!/usr/bin/env python
# Author(s): Sanil Pande and Rohan Rao
"""Calculates offset of chassis (lidar) from pod center."""

import rospy
import numpy as np

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from state_machine.msg import StateOut

# define constants related to the pod and chassis design
FRONT_LIDAR_DIST_FROM_CENTER = 1.1
FRONT_LIDAR_HEIGHT = 0.7
LENGTH_OF_POD_SHORT_SIDE = 0.95
WIDTH_OF_POD_LONG_SIDE = 1.76
HEIGHT_OF_UPPER_POD_LEG = 0.55

# define a coordinate frame with the origin centred at the front LIDAR
right_upper_pod_leg_pos = np.array((-FRONT_LIDAR_DIST_FROM_CENTER+LENGTH_OF_POD_SHORT_SIDE/2, WIDTH_OF_POD_LONG_SIDE/2))
left_upper_pod_leg_pos = np.array((-FRONT_LIDAR_DIST_FROM_CENTER+LENGTH_OF_POD_SHORT_SIDE/2, -WIDTH_OF_POD_LONG_SIDE/2))
right_lower_pod_leg_pos = np.array((-FRONT_LIDAR_DIST_FROM_CENTER-LENGTH_OF_POD_SHORT_SIDE/2, WIDTH_OF_POD_LONG_SIDE/2))
left_lower_pod_leg_pos = np.array((-FRONT_LIDAR_DIST_FROM_CENTER-LENGTH_OF_POD_SHORT_SIDE/2, -WIDTH_OF_POD_LONG_SIDE/2))

# define a radius allowance to consider around each pod leg location
Z_MIN = -0.3
Z_MAX = 0.3
POD_LEG_ESTIMATION_RADIUS_ALLOWANCE = 0.1 

class DockingVerification():
    """Check state, publish docking offset from pod."""
    def __init__(self, average_len=10):
        self.average_len = average_len
        self.moving_average = np.zeros(average_len)
        self.counter = 0
        self.docking_state = False

        self.publisher = rospy.Publisher('dock_offset', Float64, queue_size=10)
        self.lidar_sub = rospy.Subscriber('SM_output', StateOut,
                                          self.state_listener)
        self.lidar_sub = rospy.Subscriber("points_raw", PointCloud2,
                                          self.velodyne_points_callback)

    def velodyne_points_callback(self, point_cloud):
        """Estimate and publish pod center relative to lidar."""
        if self.docking_state is False: # no need to calculate
            return

        x, y, z = [], [], []
        left_upper = []
        right_upper = []
        left_lower = []
        right_lower = []
        
        for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            # check if the points are within the left upper leg
            x, y, z = p[:3]
            c0, c1 = left_upper_pod_leg_pos
            c2, c3 = right_upper_pod_leg_pos
            c4, c5 = left_lower_pod_leg_pos
            c6, c7 = right_lower_pod_leg_pos
            if((x-c0)**2 + (y-c1)**2 <= POD_LEG_ESTIMATION_RADIUS_ALLOWANCE**2):
                if(z>=Z_MIN and z<=Z_MAX):
                    left_upper.append((x,y,z))
                    cloud_points.append((x,y,z))
            elif((x-c2)**2 + (y-c3)**2 <= POD_LEG_ESTIMATION_RADIUS_ALLOWANCE**2):
                if(z>=Z_MIN and z<=Z_MAX):
                    right_upper.append((x,y,z))
                    cloud_points.append((x,y,z))
            elif((x-c4)**2 + (y-c5)**2 <= POD_LEG_ESTIMATION_RADIUS_ALLOWANCE**2):
                if(z>=Z_MIN and z<=Z_MAX):
                    left_lower.append((x,y,z))
                    cloud_points.append((x,y,z))
            elif((x-c6)**2 + (y-c7)**2 <= POD_LEG_ESTIMATION_RADIUS_ALLOWANCE**2):
                if(z>=Z_MIN and z<=Z_MAX):
                    right_lower.append((x,y,z))
                    cloud_points.append((x,y,z))

        left_upper = np.array(left_upper)
        right_upper = np.array(right_upper)
        left_lower = np.array(left_lower)
        right_lower = np.array(right_lower)

        # fit the upper left pod leg
        x, y, z = left_upper.T
        xposLU, yposLU = x.mean(), y.mean()

        # fit the upper right pod leg
        x, y, z = right_upper.T
        xposRU, yposRU = x.mean(), y.mean()

        # fit the lower left pod leg
        x, y, z = left_lower.T
        xposLL, yposLL = x.mean(), y.mean()

        # fit the lower right pod leg
        x, y, z = right_lower.T
        xposLR, yposLR = x.mean(), y.mean()

        # get the mean offset
        meanX = (xposLU+xposLR+xposRU+xposLL)/4+FRONT_LIDAR_DIST_FROM_CENTER
        meanY = (yposLU+yposLR+yposRU+yposLL)/4
        offset = np.sqrt(meanX**2 + meanY**2)

        self.moving_average[self.counter % self.average_len] = offset
        self.counter += 1

        #lidar is assumed to be at front of chassis
        self.publisher.publish(round(np.mean(self.moving_average), 4))

    def state_listener(self, state):
        """Listen to chassis state and update own state."""
        if state.CurrState == StateOut.State_D_Approach:
            self.docking_state = True
        else:
            self.docking_state = False

if __name__ == '__main__':
    rospy.init_node('docking_verification', anonymous=True)
    dv = DockingVerification()
    rospy.spin()
