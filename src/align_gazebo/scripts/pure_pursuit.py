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

def pursuitToWaypoint(waypoint, i):
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

  while target_distance > waypoint_tol[i]:

    dx = waypoint[0] - pix_bot_center.position.x
    dy = waypoint[1] - pix_bot_center.position.y
    lookahead_dist = np.sqrt(dx * dx + dy * dy)
    lookahead_theta = math.atan2(dy, dx)
    # lookahead_theta = waypoint[2]
    alpha = shortest_angular_distance(pix_bot_theta, lookahead_theta)

    

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    # Publishing constant speed of 1m/s
    cmd.drive.speed = 1

    # Reactive steering
    if alpha < 0:
      st_ang = max(-max_steering_angle[i], alpha)
    else:
      st_ang = min(max_steering_angle[i], alpha)

    cmd.drive.steering_angle = st_ang

    target_distance = math.sqrt(dx * dx + dy * dy)

    error_speed = target_distance
    if last_error == 0:
      pass
    else:
      delta_error = error_speed - last_error

    velocity = Kp * error_speed + Kd * delta_error

    MIN_VEL = 0.2
    MAX_VEL = 2

    if i == 1:
      MAX_VEL = 1.5
    elif i == 2:
      MAX_VEL = 1
    elif i == 3:
      MAX_VEL = 1
    elif i == 4:
      MAX_VEL = 0.3
      MIN_VEL = 0.05


    velocity = max(MIN_VEL, min(MAX_VEL , velocity))

    cmd.drive.speed = velocity


    cmd_pub.publish(cmd)
    rospy.wait_for_message("/align/ground_truth/state", Odometry, 5)


if __name__ == '__main__':

  rospy.init_node('pure_pursuit')
  cmd_pub = rospy.Publisher('/align/ackermann_cmd', AckermannDriveStamped, queue_size=10)

  waypoints = np.zeros((num_waypoints, 3))
  
  rospy.Subscriber("/waypoints_goal",
                   PoseArray,
                   waypointCallback)
  rospy.wait_for_message("/waypoints_goal", PoseArray)


  pix_bot_center = Pose()
  pix_bot_velocity = Twist()
  rospy.Subscriber("/align/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/align/ground_truth/state", Odometry,5)

  for i_,w in enumerate(waypoints):
    pursuitToWaypoint(w,i_)

  state = "finished"
  



