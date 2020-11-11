#!/usr/bin/env python
# license removed for brevity

# Author: Rohan Rao

import rospy
Path = 'src/align_navigation/scripts/PodLocationServer/'
import sys
sys.path.insert(1, Path)
from PodServer import *
import tf
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Float64, Int8
from angles import *
from state_machine.msg import *


import tf
pi = math.pi

last_goal = False
target_waypoint = 0

enable_p2p = False

goal_tolerance = 0.75

def vehicleStateCallback(msg):
    global pix_bot_center, pix_bot_theta
    pix_bot_center.position.x = msg.pose.pose.position.x
    pix_bot_center.position.y = msg.pose.pose.position.y
    pix_bot_center.orientation = msg.pose.pose.orientation
    pix_bot_theta = euler_from_quaternion(
    [pix_bot_center.orientation.x, pix_bot_center.orientation.y, pix_bot_center.orientation.z,
     pix_bot_center.orientation.w])[2]

    # @Rohan Since you are using goals for base_link the center will be rear_axle, if tests fail we can revert this
    # pix_bot_center.position.x, pix_bot_center.position.y = adjustRearAxletoCenter(pix_bot_center.position.x, pix_bot_center.position.y, pix_bot_theta)

def adjustCentertoRearAxle(p_x, p_y, p_theta):
  p_x = p_x - 0.95 * np.cos(p_theta)
  p_y = p_y - 0.95 * np.sin(p_theta)
  return p_x, p_y

def adjustRearAxletoCenter(p_x, p_y, p_theta):
  p_x = p_x + 0.95 * np.cos(p_theta)
  p_y = p_y + 0.95 * np.sin(p_theta)
  return p_x, p_y

def move_base_cancel_goal():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

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

def is_close():
    global target_waypoint, pix_bot_center, pix_bot_theta, goal_tolerance
    centererr = np.sqrt((target_waypoint[0]-pix_bot_center.position.x)**2+(target_waypoint[1]-pix_bot_center.position.y)**2)
    thetaerr = shortest_angular_distance(pix_bot_theta,target_waypoint[2])
    if(centererr < goal_tolerance and thetaerr < 0.25):
        return True
    else:
        return False

def stateCallback(StateInfo):
    global enable_p2p, location_target
    enable_p2p = True if StateInfo.CurrState == StateOut.State_P2P else False
    PodId = StateInfo.PodInfo
    if location_target != PodId:
        location_target = PodId
    # print('In p2p call back p2p flag ', enable_p2p)

def move_to_goal(wp_array):
    global enable_p2p, target_waypoint, pix_bot_center, pix_bot_theta, last_goal
    total_wp = len(wp_array)
    i=0
    while(i<total_wp):
        
        if enable_p2p == True:
            # goto wp i
            print("Moving to next Goal")
            target_waypoint = wp_array[i]
            movebase_client()
            # when close (within tol) or move_base_state is success or HMS error
            while not last_goal and not is_close(): #or move_base_state == SUCCESS or HMS_ERROR==True):
                if enable_p2p != True:
                    break
        
        move_base_cancel_goal()
        # also last_goal flag
        if enable_p2p == True:
            i+=1
        if i == total_wp-1:
            last_goal = True

if __name__ == '__main__':
    rospy.init_node('p2p_py')
    location_target = -1

    pix_bot_center = Pose()
    pix_bot_theta = 0

    rospy.Subscriber(ODOM_INF, Odometry, vehicleStateCallback)
    rospy.wait_for_message(ODOM_INF, Odometry,5)
    #rospy.Subscriber("/state", Int8, stateCallback)
    rospy.Subscriber("SM_output", StateOut, stateCallback)
    sm_pub = rospy.Publisher("SM_input", StateIn, queue_size=1)
    while not rospy.is_shutdown():
        if(location_target != -1 and enable_p2p == True):
            #From location_target read waypoints.npy
            '''
            Location, WaypointsFile = GetPodLocAndWaypointsFileName(Path + 'PickupPodLoc.json', str(location_target))
            waypoints = np.load(WaypointsFile)
            '''
            print("Target ID:", location_target )
            if location_target == 12:
                waypoints = np.array([[0,0,0],[10,0,0],[20,0,0], [25,0,0],[32.5,10,np.pi/2], [32.5,28,np.pi/2], [32.5,30,np.pi/2]])
            elif location_target == 3:
                waypoints = np.array([[-46.5, -25, np.pi], [-54, -18, 0], [-44,-10,np.pi/2], [-44,4,np.pi/2], [-44,20,np.pi/2], [-50.7,28,-np.pi], [-51.7,28,-np.pi]])
            else:
                print("Unknown Goal Given")
                #TODO: Send failure to SM node

            try: 
                move_to_goal(waypoints)
                print("Target Reached")
                StateUpdateMsg = StateIn()
                StateUpdateMsg.TransState = StateOut.State_P2P
                StateUpdateMsg.StateTransitionCond = 1
                sm_pub.publish(StateUpdateMsg)
                location_target = -1
            except:
                move_base_cancel_goal()
                break
