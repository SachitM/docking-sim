#!/usr/bin/env python
# license removed for brevity

# Author: Sachit Mahajan

import rospy
import tf
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

from utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Float64
from state_machine.msg import *#StateIn
#from state_machine.msg import StateOut

import tf
pi = math.pi

waypoints = []
last_goal = False
dock_error = 0
target_waypoint = 0

EnableApproach = False
EnableVerifyPose = False
EnableRetrace = False
EnableLock = False

def dock_callback(msg):
  global dock_error, state
  dock_error = msg.data
#   if state == "verifying_pose":
#     print(dock_error)

def waypointCallback(msg):
    global waypoints, last_goal
    if last_goal == True:
        return

    for i in range(len(msg.poses)):
        waypoints[i, 0] = msg.poses[i].position.x
        waypoints[i, 1] = msg.poses[i].position.y - 0.005
        waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
    global pix_bot_center, pix_bot_theta, pix_bot_velocity, state
    pix_bot_center.position.x = msg.pose.pose.position.x
    pix_bot_center.position.y = msg.pose.pose.position.y
    pix_bot_center.orientation = msg.pose.pose.orientation

    pix_bot_theta = euler_from_quaternion(
    [pix_bot_center.orientation.x, pix_bot_center.orientation.y, pix_bot_center.orientation.z,
        pix_bot_center.orientation.w])[2]

    pix_bot_center.position.x, pix_bot_center.position.y = adjustRearAxletoCenter(pix_bot_center.position.x, pix_bot_center.position.y, pix_bot_theta)
    # if(state == "finished"):
    #   print( msg.pose.pose.position.x, msg.pose.pose.position.y, pix_bot_theta)

    pix_bot_velocity.linear = msg.twist.twist.linear
    pix_bot_velocity.angular = msg.twist.twist.angular

def adjustCentertoRearAxle(p_x, p_y, p_theta):
  p_x = p_x - 0.95 * np.cos(p_theta)
  p_y = p_y - 0.95 * np.sin(p_theta)
  return p_x, p_y

def adjustRearAxletoCenter(p_x, p_y, p_theta):
  p_x = p_x + 0.95 * np.cos(p_theta)
  p_y = p_y + 0.95 * np.sin(p_theta)
  return p_x, p_y

def pursuitToWaypoint(waypoint, i):
  global pix_bot_center, pix_bot_theta, pix_bot_velocity, cmd_vel_pub, ODOM_INF
  print waypoint
  Kp = 0.9
  Kd = 0.4
  waypoint_tol_ = [0.08,0.05]
  max_acc_ = [ 0.7, 0.3]
  max_steering_angle_ = [0.3,0.1]
  MAX_VEL_ = [0.5, 0.2]
  MIN_VEL_ = [0.2, 0.05]
  MAX_VEL = MAX_VEL_[i]
  MIN_VEL = MIN_VEL_[i]
  max_acc = max_acc_[i]
  waypoint_tol = waypoint_tol_[i]
  max_steering_angle = max_steering_angle_[i]
  rospy.wait_for_message(ODOM_INF, Odometry, 5)
  dx = waypoint[0] - pix_bot_center.position.x
  dy = waypoint[1] - pix_bot_center.position.y
  target_distance = math.sqrt(dx*dx + dy*dy)

  cmd = AckermannDriveStamped()
  cmd.header.stamp = rospy.Time.now()
  cmd.header.frame_id = "base_link"
  cmd.drive.speed = pix_bot_velocity.linear.x

  cmd.drive.acceleration = max_acc

  delta_error = 0.0
  last_error = 0.0

  while target_distance > waypoint_tol:
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
      st_ang = max(-max_steering_angle, alpha)
    else:
      st_ang = min(max_steering_angle, alpha)

    cmd.drive.steering_angle = st_ang

    target_distance = math.sqrt(dx * dx + dy * dy)

    error_speed = target_distance
    if last_error == 0:
      pass
    else:
      delta_error = error_speed - last_error

    velocity = Kp * error_speed + Kd * delta_error

    velocity = max(MIN_VEL, min(MAX_VEL , velocity))
    cmd.drive.speed = velocity

    cmd_vel_pub.publish(cmd)
    rospy.wait_for_message(ODOM_INF, Odometry, 5)

  if i == 1 :
    cmd = AckermannDriveStamped() 
    cmd.drive.acceleration = 0.0
    cmd.drive.speed = 0
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    cmd.drive.steering_angle = 0
    cmd_vel_pub.publish(cmd)
    # rospy.wait_for_message(ODOM_INF, Odometry, 5)
    # cmd_vel_pub.publish(cmd)


def movebase_client():

    global target_waypoint
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target_waypoint[0]
    goal.target_pose.pose.position.y = target_waypoint[1]
    quaternion = tf.transformations.quaternion_from_euler(0,0,target_waypoint[2])
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]


   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   


def go_to_goal(goal):
    global target_waypoint
    goal[0], goal[1] = adjustCentertoRearAxle(goal[0], goal[1], goal[2])
    target_waypoint = goal
    try:
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        result = movebase_client()
        if result:
            rospy.loginfo("Waypoint execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

def docking_execution():
    
    StateUpdateMsg = StateIn()
    while not rospy.is_shutdown():
        global waypoints, pix_bot_center, pix_bot_theta, pix_bot_velocity, state, cmd_pub, sm_pub, EnableApproach, EnableLock, EnableRetrace, EnableVerifyPose
        print(EnableLock, EnableRetrace, EnableVerifyPose, EnableApproach)
        if EnableApproach:
            last_goal = False
            go_to_goal(waypoints[0])
            go_to_goal(waypoints[1])
            last_goal = True
            OFFSET = 0.03
            pursuitToWaypoint(waypoints[-2],0)
            waypoints[-1, 0] += OFFSET * np.cos(waypoints[-1, 2]) 
            waypoints[-1, 1] += OFFSET * np.sin(waypoints[-1, 2]) 
            pursuitToWaypoint(waypoints[-1],1)
            StateUpdateMsg.TransState = StateOut.State_D_Approach
            StateUpdateMsg.StateTransitionCond = 1
            sm_pub.publish(StateUpdateMsg)
            rate.sleep()
            rate.sleep()

        elif EnableVerifyPose:
            rospy.Subscriber("/dock_offset",
                            Float64,
                            dock_callback)
            # rospy.wait_for_message("/dock_offset", Float64)
            # state = "verifying_pose"
            # print(state)
            rate.sleep()
            goal = waypoints[-1] + 0
            dx = goal[0] - pix_bot_center.position.x
            dy = goal[1] - pix_bot_center.position.y
            target_distance = math.sqrt(dx*dx + dy*dy)
            diff_angle = goal[2] - pix_bot_theta
            print("Pose_Error (Estimated) (cm,degree): = ", target_distance*100, diff_angle * 180/np.pi)
            # if dock_error < 0.05:
            #     print("Dock_Error (Measured) (cm): = ", dock_error*100)
            dock_error = 0
            # print(state)
            StateUpdateMsg.TransState = StateOut.State_Verify
            if target_distance > 0.1 or dock_error > 0.20:
                # state = "retracing"
                # print(state)
                StateUpdateMsg.StateTransitionCond = 0
                rate.sleep()
                
            else:
                StateUpdateMsg.StateTransitionCond = 1
                # state = "docking"
            sm_pub.publish(StateUpdateMsg)

        
        elif EnableRetrace:
            go_to_goal(waypoints[0])
            StateUpdateMsg.TransState = StateOut.State_Retrace
            StateUpdateMsg.StateTransitionCond = 1
            sm_pub.publish(StateUpdateMsg)

        elif EnableLock:
            lift_goal = Float64()
            lift_goal.data = 0.5
            cmd_pub.publish(lift_goal)
            StateUpdateMsg.TransState = StateOut.State_Lock
            StateUpdateMsg.StateTransitionCond = 1
            sm_pub.publish(StateUpdateMsg) 
            break
         

        # state = "finished"
        # print(state)


def StateMachineCb(StateInfo):
    global EnableApproach, EnableLock, EnableRetrace, EnableVerifyPose
    EnableApproach = True if StateInfo.CurrState == StateOut.State_D_Approach else False
    EnableVerifyPose = True if StateInfo.CurrState == StateOut.State_Verify else False
    EnableRetrace = True if StateInfo.CurrState == StateOut.State_Retrace else False
    EnableLock = True if StateInfo.CurrState == StateOut.State_Lock else False
    #print("Curr state, enabled ", StateInfo.CurrState, EnableApproach)

if __name__ == '__main__':

    rospy.init_node('movebase_client_py')
    waypoints = np.zeros((num_waypoints, 3))
    rate = rospy.Rate(0.25)
    
    rospy.Subscriber("/waypoints_goal",
                   PoseArray,
                   waypointCallback)
    rospy.Subscriber("SM_output", StateOut, StateMachineCb)
    rospy.wait_for_message("/waypoints_goal", PoseArray)

    pix_bot_center = Pose()
    pix_bot_velocity = Twist()
    rospy.Subscriber(ODOM_INF,
                    Odometry, vehicleStateCallback)
    rospy.wait_for_message(ODOM_INF, Odometry,5)

    cmd_vel_pub = rospy.Publisher('/align/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    cmd_pub = rospy.Publisher('/autoware_gazebo/lift_controller/command', Float64, queue_size=1)
    sm_pub = rospy.Publisher("SM_input", StateIn, queue_size=1)
    docking_execution()

    
