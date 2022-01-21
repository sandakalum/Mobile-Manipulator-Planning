#!/usr/bin/env python
# coding: UTF-8

import rospy, os, yaml
from pypozyx import *
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class VisualizeClass(object):

    def __init__(self):
        rospy.init_node('pozyx_visualize', anonymous=True)
        self.tag_pub = rospy.Publisher("pozyx_tag_marker", Marker, queue_size=1000)
        self.anchors_pub = rospy.Publisher("pozyx_anchors_marker", MarkerArray, queue_size=1000)

        rospy.Subscriber("/pozyx", PoseStamped, self.callback_tag, queue_size=1000)

        ########## Anchors (network_id, flag, pos) ###########

        # Load config_anchors.yaml
        config_file = os.environ['HOME'] + "/catkin_ws/src/ros_tms/tms_ss/tms_ss_ninebot_pozyx/config_anchors.yaml"

        with open(config_file, 'rt') as fp:
            config_data = fp.read()

        anchors_data = yaml.safe_load(config_data)
        my_anchors = anchors_data['pozyx_anchors']

        anchors = []

        for m_anchor in my_anchors:
            dev_coor = DeviceCoordinates()
            dev_coor.network_id = m_anchor['id']
            dev_coor.flag = 1 # Anchor
            dev_coor.pos = Coordinates(m_anchor['coor_x'], m_anchor['coor_y'], 0)
            anchors.append(dev_coor)

        ################################

        self.anchors = anchors
        self.frame_id = "world_link"
        self.isNinebot = False
        self.isHuman = True

        rospy.spin()

    def callback_tag(self, data):

        marker_tag = Marker()
        marker_tag.header.frame_id = self.frame_id
        marker_tag.header.stamp = rospy.Time.now()
        marker_tag.id = 0
        marker_tag.action = Marker().ADD

        if self.isNinebot:
            marker_tag.type = Marker().MESH_RESOURCE
            marker_tag.mesh_resource = "package://tms_ss_ninebot_pozyx/meshes/ninebot_v2.dae"
            marker_tag.mesh_use_embedded_materials = True
            marker_tag.scale.x = 0.01
            marker_tag.scale.y = 0.01
            marker_tag.scale.z = 0.01
        elif self.isHuman:
            marker_tag.type = Marker().MESH_RESOURCE
            marker_tag.mesh_resource = "package://tms_ss_ninebot_pozyx/meshes/WalkingMan4.dae"
            marker_tag.mesh_use_embedded_materials = True
            marker_tag.scale.x = 0.025
            marker_tag.scale.y = 0.025
            marker_tag.scale.z = 0.025
        else:
            marker_tag.type = Marker().CYLINDER
            marker_tag.scale.x = 0.2
            marker_tag.scale.y = 0.2
            marker_tag.scale.z = 1.0
            marker_tag.color.r = 1.0
            marker_tag.color.g = 0.0
            marker_tag.color.b = 0.0
            marker_tag.color.a = 1.0

        marker_tag.pose = data.pose
        marker_tag.lifetime = rospy.Duration()
        marker_tag.pose.orientation.x = 0
        marker_tag.pose.orientation.y = 0
        marker_tag.pose.orientation.z = 0
        marker_tag.pose.orientation.w = 0

        self.publish_anchors()
        self.tag_pub.publish(marker_tag)

    def publish_anchors(self):
        markerarray = MarkerArray()

        #offset_x = 12.838
        #offset_y = 3.330

        for anchor in self.anchors:
            marker_anchor = Marker()
            marker_anchor.header.frame_id = self.frame_id
            marker_anchor.header.stamp = rospy.Time.now()
            marker_anchor.id = anchor.network_id
            marker_anchor.action = Marker().ADD
            marker_anchor.type = Marker().CYLINDER
            marker_anchor.scale.x = 0.2
            marker_anchor.scale.y = 0.2
            marker_anchor.scale.z = 1.0
            marker_anchor.color.r = 0.0
            marker_anchor.color.g = 1.0
            marker_anchor.color.b = 0.0
            marker_anchor.color.a = 1.0
            marker_anchor.lifetime = rospy.Duration()
            marker_anchor.pose.position.x = anchor.pos.x
            marker_anchor.pose.position.y = anchor.pos.y

            #marker_anchor.pose.position.x = anchor.pos.x - offset_x
            #marker_anchor.pose.position.y = -(anchor.pos.y - offset_y)
            
            markerarray.markers.append(marker_anchor)
        
        self.anchors_pub.publish(markerarray)

if __name__ == "__main__":
    try:
        VisualizeClass()
    except rospy.ROSInterruptException:
        rospy.loginfo("pozyx_localize node finished.")
