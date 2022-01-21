#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy, os, yaml
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Load config_anchors.yaml
config_file = os.environ['HOME'] + "/catkin_ws/src/ros_tms/tms_ss/tms_ss_ninebot_pozyx/config_anchors.yaml"

with open(config_file, 'rt') as fp:
    config_data = fp.read()

anchors_data = yaml.safe_load(config_data)
my_anchors = anchors_data['pozyx_anchors']

def publish_anchors():

    pub_anchors = rospy.Publisher('/pozyx_anchors_marker', MarkerArray, queue_size=10)
    markerarray = MarkerArray()

    for m_anchor in my_anchors:
        marker = Marker()
        marker.header.frame_id = "/world_link"
        marker.header.stamp    = rospy.Time.now()
        marker.id              = m_anchor['id']
        marker.type            = 3 # cylinder
        marker.action          = 0 # add
        marker.scale.x         = 0.2
        marker.scale.y         = 0.2
        marker.scale.z         = 1.0
        marker.color.r         = 1.0
        marker.color.g         = 0.0
        marker.color.b         = 1.0
        marker.color.a         = 1.0
        marker.lifetime = rospy.Duration()
        marker.pose.position.x = m_anchor['coor_x']
        marker.pose.position.y = m_anchor['coor_y']
        markerarray.markers.append(marker)

    pub_anchors.publish(markerarray)

rospy.init_node('anchor_visualize', anonymous=True)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
	    publish_anchors()
    except rospy.ROSInterruptException:
        rospy.loginfo("debug_marker_generate terminated")
