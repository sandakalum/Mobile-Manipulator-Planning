#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import math
from actionlib_msgs.msg     import GoalID
from geometry_msgs.msg      import PoseWithCovarianceStamped
from geometry_msgs.msg      import PoseStamped
from visualization_msgs.msg import Marker

class VisualizationClass:
  def __init__(self):
    rospy.init_node('ninebot_visualization', anonymous=True)

    self.portable_number = rospy.get_param('~portable_number', '1')
    self.portable_frame  = rospy.get_param('~portable_frame_id', 'pot_1')
    self.global_frame    = rospy.get_param('~global_frame_id', 'map1')
    self.base_frame      = rospy.get_param('~base_frame_id', 'base_footprint1')
    self.server_frame    = rospy.get_param('~server_frame_id', 'server_map')

    self.marker_pub1 = rospy.Publisher('/target_s_' + self.portable_frame, Marker, queue_size=10)
    self.marker_pub2 = rospy.Publisher('target_'    + self.portable_frame, Marker, queue_size=10)
    
    self.start_pub   = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    self.goal_pub    = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback_goal, queue_size=10)
    rospy.Subscriber('move_base/cancel', GoalID, self.callback_cancel, queue_size=10)
    
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.callback_rviz_start, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback_rviz_goal, queue_size=10)

    rate        = rospy.Rate(10) # 10hz
    listener    = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
      try:
        (point, quate) = listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
        broadcaster.sendTransform((point[0], point[1], 0.0), \
                   (quate[0], quate[1], quate[2], quate[3]), \
        rospy.Time.now(), self.portable_frame, self.server_frame)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass
      rate.sleep()

  def callback_goal(self, data):
    marker                  = Marker()
    marker.id               = self.portable_number
    marker.header.stamp     = rospy.Time.now()
    marker.type             = 3 #cylinder
    marker.action           = 0 #add
    marker.scale.x          = 0.2
    marker.scale.y          = 0.2
    marker.scale.z          = 1.0
    marker.color.r          = 1
    marker.color.g          = 1
    marker.color.b          = 1
    marker.color.a          = 1.0
    marker.lifetime         = rospy.Duration()
    marker.pose.position    = data.pose.position
    marker.pose.orientation = data.pose.orientation
    marker.pose.position.z  = 0.3

    marker.header.frame_id = self.server_frame
    self.marker_pub1.publish(marker)
    marker.header.frame_id = self.global_frame
    self.marker_pub2.publish(marker)
    
  def callback_cancel(self, data):
    marker        = Marker()
    marker.id     = self.portable_number
    marker.action = 2 #delete
    self.marker_pub1.publish(marker)
    self.marker_pub2.publish(marker)

  def callback_rviz_start(self, data):
    data.header.frame_id = self.global_frame
    self.start_pub.publish(data)
  
  def callback_rviz_goal(self, data):
    data.header.frame_id = self.global_frame
    self.goal_pub.publish(data)

if __name__ == '__main__':
  try:
    VisualizationClass()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_visualization node finished.")