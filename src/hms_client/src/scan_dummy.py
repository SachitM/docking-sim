#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

def call_back(msg):
    msg_c = msg
    msg_c.ranges = [0]*560
    msg_c.ranges[2] = -1
    pub.publish(msg_c)


if __name__ == '__main__':

    rospy.init_node('scan_dummy', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, call_back)
    rospy.spin()