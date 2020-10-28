#!/usr/bin/env python
# license removed for brevity

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

import tf
pi = math.pi

waypoints = []
last_goal = False
dock_error = 0
target_waypoint = 0
def dock_callback(msg):
  global dock_error, state
  dock_error = msg.data
  if state == "verifying_pose":
    print(dock_error)

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

    # if(state == "finished"):
    #   print( msg.pose.pose.position.x, msg.pose.pose.position.y, pix_bot_theta)

    pix_bot_velocity.linear = msg.twist.twist.linear
    pix_bot_velocity.angular = msg.twist.twist.angular

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
    target_waypoint = goal
    try:
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        result = movebase_client()
        if result:
            rospy.loginfo("Waypoint execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

def docking_execution():
    global waypoints, pix_bot_center, pix_bot_theta, pix_bot_velocity, state
    for i in range(num_waypoints-1):
        go_to_goal(waypoints[i])

    state = "finished"
    print(state)
    rate.sleep()
    rate.sleep()
    rospy.Subscriber("/dock_offset",
                    Float64,
                    dock_callback)
    # rospy.wait_for_message("/dock_offset", Float64)
    state = "verifying_pose"
    print(state)
    rate.sleep()
    goal = waypoints[-2] + 0
    dx = goal[0] - pix_bot_center.position.x
    dy = goal[1] - pix_bot_center.position.y
    target_distance = math.sqrt(dx*dx + dy*dy)
    diff_angle = goal[2] - pix_bot_theta
    print("Pose_Error (Estimated) (cm,degree): = ", target_distance*100, diff_angle * 180/np.pi)
    if dock_error < 0.05:
        print("Dock_Error (Measured) (cm): = ", dock_error*100)

    print(state)
    if target_distance > 0.1 or dock_error > 0.20:
        state = "retracing"
        print(state)
        rate.sleep()
        go_to_goal(waypoints[-1])

    state = "finished"
    print(state)


if __name__ == '__main__':

    rospy.init_node('movebase_client_py')
    waypoints = np.zeros((num_waypoints, 3))
    rate = rospy.Rate(0.25)
    rospy.Subscriber("/waypoints_goal",
                   PoseArray,
                   waypointCallback)
    # rospy.wait_for_message("/waypoints_goal", PoseArray)

    waypoints[0,0] = 32.5
    waypoints[0,1] = 35
    waypoints[0,2] = math.pi/2

    waypoints[1,0] = 32.5
    waypoints[1,1] = 38.13
    waypoints[1,2] = math.pi/2

    waypoints[2,0] = 32.5
    waypoints[2,1] = 34
    waypoints[2,2] = math.pi/2

    pix_bot_center = Pose()
    pix_bot_velocity = Twist()
    rospy.Subscriber(ODOM_INF,
                    Odometry, vehicleStateCallback)
    rospy.wait_for_message(ODOM_INF, Odometry,5)

    docking_execution()

    # dx = waypoints[-2,0] - pix_bot_center.position.x
    # dy = waypoints[-2,1] - pix_bot_center.position.y
    # target_distance = math.sqrt(dx*dx + dy*dy)
    # lookahead_theta = math.atan2((dy),(dx))

    # if(target_distance < 0.1):
    #     print("Already Close to Goal")
    #     state = "finished"
    # else:
    #     if abs(lookahead_theta *180/pi) > 300:
    #         print("Angle too Large")
    #         state = "finished"
    #         exit(0)
    #     else:
    #         print("Moving to 1st Waypoint")
    #         goals = waypoints[0] + 0
    #         dxs = goals[0] - pix_bot_center.position.x
    #         dys = goals[1] - pix_bot_center.position.y
    #         target_distances = math.sqrt(dxs*dxs + dys*dys)
    #         diff_angles = abs(goals[2] - pix_bot_theta)*180/np.pi
    #         if target_distances > 1 or diff_angles > 50:
    #             print("Very Far From Initial Estimate, Overriding Limits")

    