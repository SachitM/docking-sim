#!/usr/bin/env python
# Author(s): Sanil Pande
"""Samples LiDAR points in direction of movement."""

import rospy
import numpy as np

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from gazebo_msgs.msg import ModelStates
# from StateMachine.msg import StateOut
from geometry_msgs.msg import Point32


class LidarDownSampling():
    """Publish point clouds in direction of movement."""
    def __init__(self):
        self.direction = 0  #vehicle not moving

        self.publisher = rospy.Publisher('downsampled_points', PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber("velodyne_points", PointCloud2,
                                          self.velodyne_points_callback)
        self.model_sub = rospy.Subscriber("align/gazebo/model_states", ModelStates, self.velocity_callback)

    def velocity_callback(self, states):
        """Check model states and return velocity of vehicle."""
        vehicle_id = -1
        for id, name in enumerate(states.name):
            if name == "align":
                vehicle_id = id

        velocity = states.twist[vehicle_id].linear.x

        if velocity > 1e-3:
            self.direction = 1
        elif velocity < -1e-3:
            self.direction = -1
        else:
            self.direction = 0


    def velodyne_points_callback(self, point_cloud):
        """Downsample points."""
        field_names = ("x", "y", "z")
        point_list = list(pc2.read_points(point_cloud, field_names=field_names, skip_nans=True))
        points_array = np.array(point_list)

        if self.direction != 0:
            valid_idx = np.where(np.sign(points_array[:, 0]) == self.direction)
            valid_pts = points_array[valid_idx]
        else:
            valid_pts = points_array

        header = point_cloud.header
        header.stamp = rospy.Time.now()

        ds_cloud = pc2.create_cloud_xyz32(header, valid_pts)

        self.publisher.publish(ds_cloud)


if __name__ == '__main__':
    rospy.init_node('lidar_downsampler', anonymous=True)
    lds = LidarDownSampling()
    rospy.spin()
