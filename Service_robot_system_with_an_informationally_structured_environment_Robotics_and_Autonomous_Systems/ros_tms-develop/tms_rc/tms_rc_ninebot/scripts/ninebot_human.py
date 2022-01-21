#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import math

from geometry_msgs.msg  import PoseStamped
from people_msgs.msg    import People
from sensor_msgs.msg    import PointCloud

#ポールへninebotの位置を送信する際のマップの原点からの座標変換(ポールの原点は90度回転しているので変換する->修正済み)
POLE_SEND_X = 0
POLE_SEND_Y = 0
#POLE_SEND_A = -1.5708
POLE_SEND_A = 0

#ポールから受け取った座標の原点（現在はポール側で90度回してから受け取っているため変換なし）
BASE_POLE_X = 0
BASE_POLE_Y = 0
BASE_POLE_A = 0

class SensorPoleClass:
  def __init__(self):
    rospy.init_node('ninebot_human', anonymous=True)

    pose_topic_name   = rospy.get_param('~pose_topic_name',   '/ninebot_pos')
    cloud_topic_name  = rospy.get_param('~cloud_topic_name',  '/cloud_ninebot_cropped')
    people_topic_name = rospy.get_param('~people_topic_name', '/people_ninebot_cropped')

    self.global_frame = rospy.get_param('~global_frame_id', 'map1')
    self.base_frame   = rospy.get_param('~base_frame_id',   'base_footprint1')
    
    self.cloud_pub    = rospy.Publisher('cloud', PointCloud, queue_size=10)
    self.people_pub   = rospy.Publisher('/people', People, queue_size=10)

    rospy.Subscriber(cloud_topic_name, PointCloud, self.callback_cloud, queue_size=10)
    rospy.Subscriber(people_topic_name, People, self.callback_people, queue_size=10)

    position_pub = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=10)

    rate        = rospy.Rate(10) # 10hz
    listener    = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
      try:
        (point, quate) = listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
        point_x                     = point[0] - POLE_SEND_X
        point_y                     = point[1] - POLE_SEND_Y
        position                    = PoseStamped()
        position.header.stamp       = rospy.Time.now()
        position.header.frame_id    = self.base_frame
        position.pose.position.x    = point_y * math.sin(POLE_SEND_A) + point_x * math.cos(POLE_SEND_A)
        position.pose.position.y    = point_y * math.cos(POLE_SEND_A) - point_x * math.sin(POLE_SEND_A) 
        position.pose.position.z    = 0;
        position.pose.orientation.x = quate[0];
        position.pose.orientation.y = quate[1];
        position.pose.orientation.z = quate[2];
        position.pose.orientation.w = quate[3];
        position_pub.publish(position)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass
      
      quate = tf.transformations.quaternion_from_euler(0, 0, BASE_POLE_A)
      broadcaster.sendTransform((BASE_POLE_X, BASE_POLE_Y, 0), \
                     (quate[0], quate[1], quate[2], quate[3]), \
      rospy.Time.now(), "base_pole", self.global_frame)
      rate.sleep()

  def callback_cloud(self, data):
    #data.header.stamp    = rospy.Time.now()
    #data.header.frame_id = self.global_frame
    data.header.frame_id = 'base_pole'
    self.cloud_pub.publish(data)
  
  def callback_people(self, data):
    #data.header.stamp    = rospy.Time.now()
    #data.header.frame_id = self.global_frame
    data.header.frame_id = 'base_pole'
    self.people_pub.publish(data)

if __name__ == '__main__':
  try:
    SensorPoleClass()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_human node finished.")
