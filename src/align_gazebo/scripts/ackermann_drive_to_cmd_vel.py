#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
  
def ackermann_drive_callback(data):
  global wheelbase
  global twist_cmd_topic
  global frame_id
  global pub

  msg = Twist()
  #msg.header.stamp = rospy.Time.now()
  #msg.header.frame_id = frame_id
  msg.linear.x = data.drive.speed
  msg.angular.z = math.tan(data.drive.steering_angle)*msg.linear.x/wheelbase

  pub.publish(msg)

if __name__ == '__main__': 
  try:
    
    rospy.init_node('ackermann_drive_to_cmd_vel')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/align/ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 1.9)
    frame_id = rospy.get_param('~frame_id', 'base_link')
    
    #rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    #pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    rospy.Subscriber(ackermann_cmd_topic, AckermannDriveStamped, ackermann_drive_callback, queue_size=1)
    pub = rospy.Publisher(twist_cmd_topic, Twist, queue_size=1)
    rospy.loginfo("Node 'ackermann_drive_to_cmd_vel' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", ackermann_cmd_topic, twist_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
