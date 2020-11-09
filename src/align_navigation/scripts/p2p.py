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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Float64

import tf
pi = math.pi

waypoints = []
last_goal = False
target_waypoint = 0


def move_base_cancel_goal():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    client.cancel_all_goals()


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
            rospy.loginfo("[Undocking]: Waypoint execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("[Undocking]: Navigation test finished.")
    

def goal_cancel():
    move_base_cancel_goal()
    # go_to_goal([20,20,0])

if __name__ == '__main__':

    rospy.init_node('undocking_client_py')
    
    goal_cancel()

