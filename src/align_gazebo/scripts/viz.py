#!/usr/bin/env python


from common import *
import numpy as np
# Marker dims
text_offset = 0.5
text_height = 0.6
bb_offset = 5

pod_loc_pred = np.zeros(3)
pod_loc_gt = np.zeros(3)

def getBBMarker():
  phz_array = MarkerArray()
  marker = Marker()
  marker.header.frame_id = "/map"
  marker.header.stamp = rospy.get_rostime()
  marker.ns = "PHZ"
  marker.id = 0
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = 0.2
  marker.color.a = 1.0
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  pt1 = Point(x=x_lim[1] + pod_loc_pred[0], y=y_lim[1]+ pod_loc_pred[1])
  pt2 = Point(x=x_lim[0] + pod_loc_pred[0], y=y_lim[1]+ pod_loc_pred[1])
  pt3 = Point(x=x_lim[0] + pod_loc_pred[0], y=y_lim[0]+ pod_loc_pred[1])
  pt4 = Point(x=x_lim[1] + pod_loc_pred[0], y=y_lim[0]+ pod_loc_pred[1])
  marker.points.append(pt1)
  marker.points.append(pt2)
  marker.points.append(pt3)
  marker.points.append(pt4)
  marker.points.append(pt1)
  marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
  phz_array.markers.append(marker)

  marker = Marker()
  marker.header.frame_id = "/map"
  marker.header.stamp = rospy.get_rostime()
  marker.ns = "PHZ_GT"
  marker.id = 1
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = 0.3
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  pt1 = Point(x=x_lim[1] + pod_loc_gt[0], y=y_lim[1]+ pod_loc_gt[1])
  pt2 = Point(x=x_lim[0] + pod_loc_gt[0], y=y_lim[1]+ pod_loc_gt[1])
  pt3 = Point(x=x_lim[0] + pod_loc_gt[0], y=y_lim[0]+ pod_loc_gt[1])
  pt4 = Point(x=x_lim[1] + pod_loc_gt[0], y=y_lim[0]+ pod_loc_gt[1])
  marker.points.append(pt1)
  marker.points.append(pt2)
  marker.points.append(pt3)
  marker.points.append(pt4)
  marker.points.append(pt1)
  
  phz_array.markers.append(marker)

  return phz_array

def getWaypointsMarker( waypoints):
  arrow_array = MarkerArray()
  i = 0
  for w in waypoints: 
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    if i == 0:
      marker.ns = "pod ground truth"
    elif i == 1:
      marker.ns = "chassis ground truth"
    elif i == 2:
      marker.ns = "pod predicted"
    else:
      marker.ns = "chassis predicted"
    marker.id = i
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, w[2]))
    marker.pose.position.x = w[0]
    marker.pose.position.y = w[1]
    marker.pose.position.z = 0
    arrow_array.markers.append(marker)
    i+=1
  return arrow_array


if __name__ == '__main__':
  rospy.init_node('marker_viz_node')

  # Subscribe to the randomly generated waypoints
  waypoints = np.zeros((4, 3))
  waypoints[0,0] = 4
  waypoints[1,0] = 3
  waypoints[2,0] = 2
  waypoints[3,0] = 1
  
  waypoint_arrow_marker = getWaypointsMarker(waypoints)
  waypoints_arrow_pub = rospy.Publisher('/estimatedLoc', MarkerArray, queue_size=10)
  phz_pub = rospy.Publisher('phz', MarkerArray, queue_size=10)

  #Make wait till one waypoint reading received
    
  print(waypoint_arrow_marker)
  r = rospy.Rate(10)  # 10hz
  

  while(True):
    pod_loc_pred = waypoints[2]
    pod_loc_gt = waypoints[0]
    phz_array = getBBMarker()
  
    phz_pub.publish(phz_array)
    waypoints_arrow_pub.publish(waypoint_arrow_marker)
    r.sleep()

