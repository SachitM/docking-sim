# import sensor_msgs.point_cloud2 as pc2

# def pointcloud2_to_array(cloud_msg, squeeze=True):
#     ''' Converts a rospy PointCloud2 message to a numpy recordarray

#     Reshapes the returned array to have shape (height, width), even if the height is 1.

#     The reason for using np.fromstring rather than struct.unpack is speed... especially
#     for large point clouds, this will be <much> faster.
#     '''
#     # construct a numpy record type equivalent to the point type of this cloud
#     dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

#     # parse the cloud into an array
#     cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

#     # remove the dummy fields that were added
#     cloud_arr = cloud_arr[
#         [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

#     if squeeze and cloud_msg.height == 1:
#         return np.reshape(cloud_arr, (cloud_msg.width,))
#     else:
#         return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 

# def on_scan(self, scan):
#     rospy.loginfo("Got scan, projecting")
#     cloud = self.laser_projector.projectLaser(scan)
#     gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
#     self.xyz_generator = gen

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

do_it = 1
def callback(data):
    global do_it
    if do_it == 1:
        pc = np.array(data)
        print(pc.shape)
        points=np.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        # np.savez('data_tes.npy', points)
        do_it = 0



if __name__ == '__main__':
    rospy.init_node('Listener', anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback)
    rospy.spin()