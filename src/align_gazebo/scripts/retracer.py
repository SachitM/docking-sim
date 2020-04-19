#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *

import tf


state = "starting"

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
  # print("Driving")
  global pix_bot_center, pix_bot_theta, pix_bot_velocity, state
  pix_bot_center.position.x = msg.pose.pose.position.x
  pix_bot_center.position.y = msg.pose.pose.position.y
  pix_bot_center.orientation = msg.pose.pose.orientation

  pix_bot_theta = euler_from_quaternion(
    [pix_bot_center.orientation.x, pix_bot_center.orientation.y, pix_bot_center.orientation.z,
     pix_bot_center.orientation.w])[2]

  if(state == "finished"):
    print( msg.pose.pose.position.x, msg.pose.pose.position.y, pix_bot_theta)


  pix_bot_velocity.linear = msg.twist.twist.linear
  pix_bot_velocity.angular = msg.twist.twist.angular

def perform_retrace(waypoint, i):
  
  print waypoint
  
  global pix_bot_center, pix_bot_theta, pix_bot_velocity, cmd_pub
  rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)
  dx = waypoint[0] - pix_bot_center.position.x
  dy = waypoint[1] - pix_bot_center.position.y
  target_distance = math.sqrt(dx*dx + dy*dy)

  cmd = AckermannDriveStamped()
  cmd.header.stamp = rospy.Time.now()
  cmd.header.frame_id = "base_link"
  cmd.drive.speed = pix_bot_velocity.linear.x

  cmd.drive.acceleration = max_acc[i]

  delta_error = 0.0
  last_error = 0.0

  while target_distance > retrace_waypoint_tol:

    dx = pix_bot_center.position.x - waypoint[0]
    dy = pix_bot_center.position.y - waypoint[1]
    lookahead_dist = np.sqrt(dx * dx + dy * dy)
    lookahead_theta = math.atan2(abs(dy), abs(dx))
    # lookahead_theta = waypoint[2]
    alpha = shortest_angular_distance(lookahead_theta, pix_bot_theta)
    print(pix_bot_theta, lookahead_theta )
    

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    # Publishing constant speed of 1m/s
    cmd.drive.speed = 1

    # Reactive steering

    print("ALPHA")
    print(alpha)

    if alpha < 0:
      st_ang = max(-max_steering_angle_ret, alpha)
    else:
      st_ang = min(max_steering_angle_ret, alpha)

    cmd.drive.steering_angle = st_ang
    target_distance = math.sqrt(dx * dx + dy * dy)

    error_speed = target_distance
    if last_error == 0:
      pass
    else:
      delta_error = error_speed - last_error

    velocity = Kp * error_speed + Kd * delta_error

    MIN_VEL = 0.1
    MAX_VEL = 0.75

    velocity = max(MIN_VEL, min(MAX_VEL , velocity))

    cmd.drive.speed = -velocity


    cmd_pub.publish(cmd)
    rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)


if __name__ == '__main__':

  rospy.init_node('pure_pursuit')
  cmd_pub = rospy.Publisher('/align/ackermann_cmd', AckermannDriveStamped, queue_size=10)

  waypoints = np.zeros((num_waypoints, 3))
  
  waypoints[0,0] = 0.5
  waypoints[1,0] = 1
  waypoints[2,0] = 2
  waypoints[3,0] = 3
  waypoints[4,0] = 4.0

  define_error_theta = 0.2 

  # rospy.Subscriber("/ackermann_vehicle/waypoints",
  #                  PoseArray,
  #                  waypointCallback)
  # rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)


  pix_bot_center = Pose()
  pix_bot_velocity = Twist()
  rospy.Subscriber("/align/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)


  goal = waypoints[-1,-1]
  state = "verifying_pose"
  dx = goal[0] - pix_bot_center.position.x
  dy = goal[1] - pix_bot_center.position.y
  target_distance = math.sqrt(dx*dx + dy*dy)

  state = "retracing"
  perform_retrace(waypoints[0],4)

  # for i_,w in enumerate(waypoints):
    # pursuitToWaypoint(w,i_)

  state = "finished"





