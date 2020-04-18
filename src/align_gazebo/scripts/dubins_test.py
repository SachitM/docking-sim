#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *

import tf
import dubins



# pid paramaters

KP = 0.95
KD = 0.01
KI = 0.01

error_history = []
error_change = None  

time_interval = 0.001 

def get_steer_pid( waypoint ):

	global rear_axle_center, rear_axle_theta , error_change , error_history

	dx = waypoint[0] - rear_axle_center.position.x
	dy = waypoint[1] - rear_axle_center.position.y
	lookahead_theta = math.atan2(dy, dx)
	err  = shortest_angular_distance(rear_axle_theta, lookahead_theta)

	error_history.append( err )
	error_history = error_history[: 8 ]

	if len(error_history) >= 2:
		diff  = (error_history[-1] - error_history[-2]) / time_interval
		integral = np.array(error_history).sum() * time_interval
	else:
		diff = 0
		integral = 0

	return (KP* err + KD*diff + KI*integral)




def get_state_vec( msg  ):

	return np.array([
			msg.pose.pose.position.x , 
			msg.pose.pose.position.y , 
			msg.pose.pose.position.z , 

			msg.pose.pose.orientation.x , 
			msg.pose.pose.orientation.y , 
			msg.pose.pose.orientation.z , 
			msg.pose.pose.orientation.w ,

			msg.twist.twist.linear.x , 
			msg.twist.twist.linear.y , 
			msg.twist.twist.linear.z , 

			msg.twist.twist.angular.x , 
			msg.twist.twist.angular.y , 
			msg.twist.twist.angular.z 

		]) , msg.header.stamp.to_nsec()





def send_cmd( speed , steer ):
	cmd = AckermannDriveStamped()

	cmd.drive.acceleration = max_acc
	cmd.header.stamp = rospy.Time.now()
	cmd.header.frame_id = "base_link"
	cmd.drive.speed = speed
	cmd.drive.steering_angle = steer
	cmd_pub.publish(cmd)




def start_rand_controls():
	ang = 0	

	while True:

		ang +=  random.uniform(-0.05 , 0.05 )
		ang = np.clip( ang , -0.6 , 0.6 )
		spd  = random.uniform(8 , 10 )
		send_cmd( spd , ang )
		rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)





def waypointCallback(msg):
	global waypoints
	for i in range(len(msg.poses)):
		waypoints[i, 0] = msg.poses[i].position.x
		waypoints[i, 1] = msg.poses[i].position.y
		waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]


def vehicleStateCallback(msg):
	print("Driving")
	global rear_axle_center, rear_axle_theta, rear_axle_velocity
	rear_axle_center.position.x = msg.pose.pose.position.x
	rear_axle_center.position.y = msg.pose.pose.position.y
	rear_axle_center.orientation = msg.pose.pose.orientation

	rear_axle_theta = euler_from_quaternion(
		[rear_axle_center.orientation.x, rear_axle_center.orientation.y, rear_axle_center.orientation.z,
		 rear_axle_center.orientation.w])[2]

	rear_axle_velocity.linear = msg.twist.twist.linear
	rear_axle_velocity.angular = msg.twist.twist.angular


def mark_straight(pts ):
	mask = np.zeros(len(pts))
	for i in range(1 , len(pts)-1):
		p0 = pts[i-1]
		p1 = pts[i]
		p2 = pts[i+1]
		
		dx = p0[0] - p1[0]
		dy = p0[1] - p1[1]
		a1 = math.atan2(dy, dx)
		
		dx = p1[0] - p2[0]
		dy = p1[1] - p2[1]
		a2 = math.atan2(dy, dx)
		
		if abs(a1 - a2) < 0.01:
			mask[i] = 1 
	mask[:-1] =  mask[1:]*mask[:-1]
	mask[1:] =  mask[1:]*mask[:-1]
	return mask
			

def _pursuitToWaypoint(waypoint , v=1 ):
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

	dx = waypoint[0] - rear_axle_center.position.x
	dy = waypoint[1] - rear_axle_center.position.y

	alpha = get_steer_pid( waypoint )

	cmd.header.stamp = rospy.Time.now()
	cmd.header.frame_id = "base_link"
	

	if alpha < 0:
		st_ang = max(-max_steering_angle, alpha)
	else:
		st_ang = min(max_steering_angle, alpha)

	cmd.drive.steering_angle = st_ang

	cmd.drive.speed = v 


	target_distance = math.sqrt(dx * dx + dy * dy)
	if(target_distance <1.5):
		cmd.drive.speed *= target_distance/1.2
	cmd_pub.publish(cmd)
	rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)



def pursuitToWaypoint(waypoint):
	q0 = ( rear_axle_center.position.x , rear_axle_center.position.y   ,  rear_axle_theta  )
	q1 = ( waypoint[0] , waypoint[1] ,  waypoint[2]  )
	turning_radius = 2.0
	step_size = 1

	path = dubins.shortest_path(q0, q1, turning_radius)
	configurations, _ = path.sample_many(step_size)

	curve_pts = mark_straight( configurations )

	np.save("w.npy" , waypoints )
	np.save("c.npy" , configurations)

	for i , c in enumerate(configurations):
		if curve_pts[i] <0.5:
			v = 1
		else :
			v = 3
		_pursuitToWaypoint([c[0] , c[1]] , v )





if __name__ == '__main__':

	rospy.init_node('dubins_path')
	cmd_pub = rospy.Publisher('/align/ackermann_cmd', AckermannDriveStamped, queue_size=10)

	waypoints = np.zeros((2, 3))
	waypoints[0,0] = 0
	waypoints[1,0] = 6

	# waypoints = np.zeros((num_waypoints, 3))
	# rospy.Subscriber("/ackermann_vehicle/waypoints",
	# 								 PoseArray,
	# 								 waypointCallback)
	# rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)

	print( waypoints )
	rear_axle_center = Pose()
	rear_axle_velocity = Twist()
	rospy.Subscriber("/align/ground_truth/state",
									 Odometry, vehicleStateCallback)
	rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)

	for w in waypoints:
		pursuitToWaypoint(w)




