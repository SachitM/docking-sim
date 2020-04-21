#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float64

moving_average = np.zeros(20)
counter = 0

def docking_offset():
    pub = rospy.Publisher('dock_offset', Float64, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(round(np.mean(moving_average), 4))
        rate.sleep()

def velodyne_points_callback(point_cloud):
    global counter
    x = []
    y = []
    z = []
    points = np.zeros(3)
    for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
        # print (" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
        if abs(p[0]) < 15 and abs(p[1]) < 15 and abs(p[2]) < 0.05:
            # points = np.append(points, np.array([p[0], p[1], p[2]]), axis = 0)
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])

    points = np.zeros((3, len(x)))
    points[0,:] = np.array(x)
    points[1,:] = np.array(y)
    points[1,:] = np.array(z)

    # print(len(points[0]))

    mean_x = np.mean(points[0])
    mean_y = np.mean(points[1])
    offset = np.sqrt(mean_x**2 + mean_y**2)

    moving_average[counter%20] = offset
    counter += 1
    # print("Offset ", mean_x, mean_y)
    # moving_average.append(offset)
    # print("MA = ", round(sum(moving_average) / len(moving_average), 3 ) )
    # print("MA = ", round(np.mean(moving_average), 4))

def listener():

    return None

if __name__ == '__main__':
    print("JMD")
    print("chill", pc2)
    rospy.init_node('docking_verification', anonymous=True)

    flag_useval = 0
    rospy.Subscriber("velodyne_points", PointCloud2, velodyne_points_callback)
    docking_offset()

    rospy.spin()
    listener()
