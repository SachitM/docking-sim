#!/usr/bin/env python


from common import *

# Marker dims
text_offset = 0.5
text_height = 0.6
bb_offset = 5

def getBBMarker():
  bb_array = MarkerArray()
  marker = Marker()
  marker.header.frame_id = "/map"
  marker.header.stamp = rospy.get_rostime()
  marker.ns = "bb_safe"
  marker.id = 0
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = 0.2
  marker.color.a = 1.0
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  pt1 = Point(x=x_lim[1], y=y_lim[1])
  pt2 = Point(x=x_lim[0], y=y_lim[1])
  pt3 = Point(x=x_lim[0], y=y_lim[0])
  pt4 = Point(x=x_lim[1], y=y_lim[0])
  marker.points.append(pt1)
  marker.points.append(pt2)
  marker.points.append(pt3)
  marker.points.append(pt4)
  marker.points.append(pt1)
  bb_array.markers.append(marker)

  marker = Marker()
  marker.header.frame_id = "/map"
  marker.header.stamp = rospy.get_rostime()
  marker.ns = "bb_ext"
  marker.id = 0
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = 0.2
  marker.color.a = 1.0
  marker.color.r = 0.0
  marker.color.g = 0.0
  marker.color.b = 1.0
  pt1 = Point(x=x_lim[1]+bb_offset, y=y_lim[1]+bb_offset)
  pt2 = Point(x=x_lim[0]-bb_offset, y=y_lim[1]+bb_offset)
  pt3 = Point(x=x_lim[0]-bb_offset, y=y_lim[0]-bb_offset)
  pt4 = Point(x=x_lim[1]+bb_offset, y=y_lim[0]-bb_offset)
  marker.points.append(pt1)
  marker.points.append(pt2)
  marker.points.append(pt3)
  marker.points.append(pt4)
  marker.points.append(pt1)
  bb_array.markers.append(marker)

  return bb_array

def getWaypointsMarker(waypoints):
  arrow_array = MarkerArray()
  i=0
  for w in waypoints:
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "waypoint_arrow"
    marker.id = i
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, w[2]))
    marker.pose.position.x = w[0]
    marker.pose.position.y = w[1]
    marker.pose.position.z = 0
    arrow_array.markers.append(marker)
    i+=1

  text_array = MarkerArray()
  i=0
  for w in waypoints:
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "waypoint_text"
    marker.id = i
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.z = text_height
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = w[0] - text_offset
    marker.pose.position.y = w[1] - text_offset
    marker.pose.position.z = 0
    marker.text = str(i+1)
    text_array.markers.append(marker)
    i+=1

  return arrow_array, text_array

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]


if __name__ == '__main__':
  rospy.init_node('marker_viz_node')

  # Subscribe to the final waypoint
  waypoint = np.zeros((1, 3))
  rospy.Subscriber("/align/pod_center",
                   PoseArray,
                   waypointCallback)
  
  # Viz marker publishers
  bb_pub = rospy.Publisher('bb', MarkerArray, queue_size=10)
  waypoints_arrow_pub = rospy.Publisher('waypoint_arrow', MarkerArray, queue_size=10)
  waypoints_text_pub = rospy.Publisher('waypoint_text', MarkerArray, queue_size=10)

  bb_marker = getBBMarker()
  waypoint_arrow_marker, waypoint_text_marker = getWaypointsMarker(waypoints)

  # Publish bounding boxes
  r = rospy.Rate(10)  # 10hz
  while bb_pub.get_num_connections() == 0:
    r.sleep()
  bb_pub.publish(bb_marker)

  # Publish waypoint poses using arrow
  while waypoints_arrow_pub.get_num_connections() == 0:
    r.sleep()
  waypoints_arrow_pub.publish(waypoint_arrow_marker)

  # Publish waypoint ids in the order to be followed
  while waypoints_text_pub.get_num_connections() == 0:
    r.sleep()
  waypoints_text_pub.publish(waypoint_text_marker)

  rospy.spin()

