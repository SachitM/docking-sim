#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

from common_pv import *
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose, Twist, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *

from bezier import *

import tf

def waypointCallback(msg):
	global waypoints
	for i in range(len(msg.poses)):
		waypoints[i, 0] = msg.poses[i].position.x
		waypoints[i, 1] = msg.poses[i].position.y
		waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def generate_path(waypts, offset, n_points):
	global path, dt, ddt, control_points, curve, path_to_pub
	
	for i in range(waypoints.shape[0]-1):
	
		p, cp = bezier_path(waypts[i,0], waypts[i,1], waypts[i,2], waypts[i+1,0], waypts[i+1,1], waypts[i+1,2], offset, n_points)
		d_cp = bezier_derivatives_control_points(cp, 2)
		
		d = []	
		dd = []
		c = []
		poses = []

		for t in np.linspace(0, 1, n_points):
			d.append(bezier_point(t, d_cp[1]))
			dd.append(bezier_point(t, d_cp[2]))
			c.append(min(1/min_turning_radius,curvature(d[-1][0], d[-1][1],dd[-1][0], dd[-1][1])))
			pose = PoseStamped()
			pose.header = path_to_pub.header
			pose.pose.position.x = bezier_point(t, cp)[0]
			pose.pose.position.y = bezier_point(t, cp)[1]
			path_to_pub.poses.append(pose)

		if (i == 0):
			path = p
			control_points = cp
			dt = np.array(d)
			ddt = np.array(dd)
			curve = np.array(c)
		else:
			path = np.vstack((path, p[1:,:]))
			control_points = np.vstack((control_points, cp[1:,:]))
			dt = np.vstack((dt,np.array(d)[1:,:]))
			ddt = np.vstack((ddt,np.array(dd)[1:,:]))
			curve = np.hstack((curve,np.array(c)[1:]))

def generate_cmd():
	global path, dt, ddt, control_points, curve
	global st_angles, velocities, accelerations, time_durations, waypoint_diff
	steering = []
	vel = []
	acc = []
	ds = []
	time = []
	time.append(0)
	ds.append(0)

	for i in range(curve.shape[0]):
		theta = max(-max_steering_angle, np.arctan(wheelbase*curve[i]))
		steering.append(min(theta, max_steering_angle))
		vel.append(min(max_velocity, np.sqrt(dt[i][0]**2+dt[i][1]**2)))
		acc.append(min(max_acc, np.sqrt(ddt[i][0]**2+ddt[i][1]**2)))
		
		if (i>0):
			ds.append(np.hypot(path[i,0]-path[i-1,0], path[i,1]-path[i-1,1]))
			time.append(2*ds[-1]/(vel[i]+vel[i-1]))
	
	st_angles = np.array(steering)
	velocities = np.array(vel)
	accelerations = np.array(acc)
	waypoint_diff = np.array(ds)
	time_durations = np.array(time)

def publish_cmd(msg):
	global path, dt, ddt, control_points, curve
	global st_angles, velocities, accelerations, time_durations, waypoint_diff
	global count
	global cmd
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	theta = msg.pose.pose.orientation
	theta = euler_from_quaternion([theta.x, theta.y, theta.z,theta.w])[2]
	
	if (count<path.shape[0]):
		angle_des = np.arctan(dt[count][1]/dt[count][0])

		if (dt[count][0]<0):
			if (dt[count][1]>=0):
				angle_des = np.pi + angle_des
			else:
				angle_des = np.pi - angle_des

		err_pos = np.hypot(x-path[count][0], y-path[count][1])
		err_angle = abs(angle_des-theta)	
		if (err_pos <= waypoint_tol):
			cmd.header.frame_id = "base_link"
			cmd.header.stamp = rospy.Time.now()
			cmd.drive.speed = velocities[count]
			count+=1
			cmd.drive.steering_angle = st_angles[count]
		
	cmd_pub.publish(cmd)


def vehicleStateCallback(msg):
	global rear_axle_center, rear_axle_theta, rear_axle_velocity
	rear_axle_center.position.x = msg.pose.pose.position.x
	rear_axle_center.position.y = msg.pose.pose.position.y
	rear_axle_center.orientation = msg.pose.pose.orientation

	rear_axle_theta = euler_from_quaternion([rear_axle_center.orientation.x, rear_axle_center.orientation.y, rear_axle_center.orientation.z,
	 rear_axle_center.orientation.w])[2]

	rear_axle_velocity.linear = msg.twist.twist.linear
	rear_axle_velocity.angular = msg.twist.twist.angular

def pursuitToWaypoint(waypoint):
	print waypoint
	global rear_axle_center, rear_axle_theta, rear_axle_velocity, cmd_pub
	rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)
	dx = waypoint[0] - rear_axle_center.position.x
	dy = waypoint[1] - rear_axle_center.position.y
	target_distance = math.sqrt(dx*dx + dy*dy)

	cmd = AckermannDriveStamped()
	cmd.header.stamp = rospy.Time.now()
	cmd.header.frame_id = "base_link"
	cmd.drive.speed = rear_axle_velocity.linear.x
	cmd.drive.acceleration = max_acc
	while target_distance > waypoint_tol:

		path_gen_pub.publish(path_to_pub)	
		dx = waypoint[0] - rear_axle_center.position.x
		dy = waypoint[1] - rear_axle_center.position.y
		lookahead_dist = np.sqrt(dx * dx + dy * dy)
		lookahead_theta = math.atan2(dy, dx)
		alpha = shortest_angular_distance(rear_axle_theta, lookahead_theta)

		cmd.header.stamp = rospy.Time.now()
		cmd.header.frame_id = "base_link"
		# Publishing constant speed of 1m/s
		cmd.drive.speed = 1

		# Reactive steering
		if alpha < 0:
			st_ang = max(-max_steering_angle, alpha)
		else:
			st_ang = min(max_steering_angle, alpha)

		cmd.drive.steering_angle = st_ang

		target_distance = math.sqrt(dx * dx + dy * dy)

		cmd_pub.publish(cmd)
		rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)

		

if __name__ == '__main__':

	rospy.init_node('bezier_path_generation')

	offset = 1
	n_points = 15
	count = 0
	cmd = AckermannDriveStamped()
	path_to_pub = Path()

	path_to_pub.header.frame_id = "map"
	path_to_pub.header.stamp = rospy.Time.now()
	
	cmd_pub = rospy.Publisher('/align/ackermann_cmd', AckermannDriveStamped, queue_size=10)
	path_gen_pub = rospy.Publisher('/path', Path, queue_size=10)

	rear_axle_center = Pose()
	rear_axle_velocity = Twist()
	
	rospy.Subscriber("/align/ground_truth/state",Odometry, vehicleStateCallback)
	rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)



	waypoints = np.zeros((5, 3))
	
	waypoints[0,0] = 0.5
	waypoints[1,0] = 1
	waypoints[2,0] = 2
	waypoints[3,0] = 3
	waypoints[4,0] = 4.0

	waypoints = np.vstack((np.array([rear_axle_center.position.x,rear_axle_center.position.y,rear_axle_theta]).reshape((1,3)), waypoints))
	
	generate_path(waypoints,offset,n_points)
	
	'''
	#Uncomment this to see the effect Ackermann cmds generated using the trajectory
	#comment out rest of the code below
	generate_cmd()
	state_sub = rospy.Subscriber("/ackermann_vehicle/ground_truth/state", Odometry, publish_cmd)
	path_gen_pub.publish(path_to_pub)
	rospy.spin()
	'''

	#Following path using pure pursuit control
	time_start = rospy.Time.now()
	total_time = rospy.Duration.from_sec(0.0)
	for w in path:
		pursuitToWaypoint(w)
		total_time +=  rospy.Time.now()-time_start
		time_start = rospy.Time.now()
		rospy.loginfo(total_time.to_sec())
		
