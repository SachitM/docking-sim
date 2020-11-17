#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image
import message_filters

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def im_cb(image_msg):
    global publisher, camera_info_msg
    # print("Time", rospy.Time.now())
    camera_info_msg.header.stamp = image_msg.header.stamp#rospy.Time.now()
    camera_info_msg.header.frame_id = 'camera'
    # image_msg.header.stamp = rospy.Time.now()
    publisher.publish(camera_info_msg)
    publisher2.publish(image_msg)

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    filename = "/home/sachit/Desktop/ROS_data/align/src/align_relative_localization/pod_localizer/scripts/ost.yaml"

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    publisher2 = rospy.Publisher("image_raw", Image, queue_size=10)
    rospy.Subscriber("/camera0/image_raw", Image, im_cb)
    rate = rospy.Rate(10)
    rospy.spin()
    # Run publisher
    # while not rospy.is_shutdown():
        
    #     rate.sleep()
