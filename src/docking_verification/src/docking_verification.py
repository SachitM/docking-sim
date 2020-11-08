#!/usr/bin/env python
"""Calculates offset of chassis (lidar) from pod center."""

import rospy
import numpy as np

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from StateMachine.msg import StateOut


class DockingVerification():
    """Check state, publish docking offset from pod."""
    def __init__(self, average_len=10):
        self.average_len = average_len
        self.moving_average = np.zeros(average_len)
        self.counter = 0
        self.docking_state = False

        self.publisher = rospy.Publisher('dock_offset', Float64, queue_size=10)
        self.lidar_sub = rospy.Subscriber('system_status', StateOut,
                                          self.state_listener)
        self.lidar_sub = rospy.Subscriber("velodyne_points", PointCloud2,
                                          self.velodyne_points_callback)

    def velodyne_points_callback(self, point_cloud):
        """Estimate and publish pod center relative to lidar."""
        if self.docking_state is False: # no need to calculate
            return

        x = y = z = []
        for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            if abs(p[0]) < 15 and abs(p[1]) < 15 and abs(p[2]) < 0.05:
                x.append(p[0])
                y.append(p[1])
                # z.append(p[2])

        points = np.zeros((3, len(x)))
        points[0, :] = np.array(x)
        points[1, :] = np.array(y)
        # points[1, :] = np.array(z)

        mean_x = np.mean(points[0])
        mean_y = np.mean(points[1])
        offset = np.sqrt(mean_x**2 + mean_y**2)

        self.moving_average[self.counter % self.average_len] = offset
        self.counter += 1

        #lidar is assumed to be at chassis center
        self.publisher.publish(round(np.mean(self.moving_average), 4))

    def state_listener(self, state):
        """Listen to chassis state and update own state."""
        if state.DestState == 4:    #TODO: Add in more states?
            self.docking_state = True
        else:
            self.docking_state = False


if __name__ == '__main__':
    rospy.init_node('docking_verification', anonymous=True)
    dv = DockingVerification()
    rospy.spin()
