#!/usr/bin/env python
# Author(s): Sanil Pande
"""Downsamples LiDAR points to speed up processing."""

import rospy
import numpy as np

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
# from StateMachine.msg import StateOut


class LidarDownSampling():
    """Check state, publish docking offset from pod."""
    def __init__(self):
        self.direction = 1  #front

        self.publisher = rospy.Publisher('downsampled_points', PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber("velodyne_points", PointCloud2,
                                          self.velodyne_points_callback)

    def velodyne_points_callback(self, point_cloud):
        """Downsample points."""

        field_names = ("x", "y", "z")
        point_list = list(pc2.read_points(point_cloud, field_names=field_names, skip_nans=True))
        points_array = np.array(point_list)

        valid_idx = np.where(np.sign(points_array[:, 0]) == self.direction)
        valid_pts = points_array[valid_idx]

        # print(valid_pts.shape)

        header = point_cloud.header
        header.stamp = rospy.Time.now()

        ds_cloud = pc2.create_cloud_xyz32(header, valid_pts)

        self.publisher.publish(ds_cloud)

if __name__ == '__main__':
    rospy.init_node('lidar_downsampler', anonymous=True)
    lds = LidarDownSampling()
    rospy.spin()
