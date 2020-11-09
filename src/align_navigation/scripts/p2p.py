#!/usr/bin/env python
# license removed for brevity

# Author: Rohan Rao

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

last_goal = False
target_waypoint = 0

def vehicleStateCallback(msg):
    global pix_bot_center
    pix_bot_center.position.x = msg.pose.pose.position.x
    pix_bot_center.position.y = msg.pose.pose.position.y
    pix_bot_center.orientation = msg.pose.pose.orientation
    pix_bot_theta = euler_from_quaternion(
    [pix_bot_center.orientation.x, pix_bot_center.orientation.y, pix_bot_center.orientation.z,
     pix_bot_center.orientation.w])[2]

    pix_bot_center.position.x = pix_bot_center.position.x + 0.95 * np.cos(pix_bot_theta)
    pix_bot_center.position.y = pix_bot_center.position.y + 0.95 * np.sin(pix_bot_theta)

def move_base_cancel_goal():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    client.cancel_all_goals()

def movebase_client():

    global target_waypoint, last_goal
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

    if last_goal:
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return client.get_result()   

def move_to_goal(wp_array):
    global target_waypoint, pix_bot_center, pix_bot_theta, last_goal
    total_wp = len(wp_array)
    i=0
    while(i<total_wp):
        # goto wp i
        target_waypoint = wp_array[i]
        # if state is approach:
        movebase_client()
        # when close (within tol) or move_base_state is success or HMS error
        while (np.sqrt((target_waypoint[0]-pix_bot_center.position.x)**2+(target_waypoint[1]-pix_bot_center.position.y)**2) > 0.5): #or move_base_state == SUCCESS or HMS_ERROR==True):
            #if state not approach
            #break
            pass
        print("Moving to next Goal")
        move_base_cancel_goal()
        # also last_goal flag
        if i == total_wp-1:
            last_goal = True
        # if state is approach:
        i+=1

if __name__ == '__main__':
    rospy.init_node('undocking_client_py')
    new_user_input_flag = 1

    pix_bot_center = Pose()
    pix_bot_theta = Twist()

    rospy.Subscriber(ODOM_INF,
                    Odometry, vehicleStateCallback)
    rospy.wait_for_message(ODOM_INF, Odometry,5)

    while True:
        if(new_user_input_flag): #and IS_APPROACH):
            waypoints = np.array([[0,0,0],[10,0,0],[20,0,0], [25,0,0],[32.5,10,np.pi/2], [32.5,30,np.pi/2]])
            move_to_goal(waypoints)
            new_user_input_flag = False
            #try catch adn stop

