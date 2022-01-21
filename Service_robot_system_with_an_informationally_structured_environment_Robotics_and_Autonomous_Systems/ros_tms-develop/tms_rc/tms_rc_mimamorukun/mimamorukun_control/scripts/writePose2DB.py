#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
/mimamorukun/odomのデータをTMS_DBに書き込むノード
"""

import datetime

import rospy
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf


rospy.init_node('writePose2DB')
db_pub = rospy.Publisher('tms_db_data', TmsdbStamped, queue_size=10)


def callback(arg):
    rospy.loginfo("get callback")
    msg = TmsdbStamped()
    msg.header.frame_id = "world_link"
    msg.header.stamp = rospy.get_rostime() + rospy.Duration(9 * 60 * 60)
    now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
    msg.tmsdb.append(Tmsdb())
    msg.tmsdb[0].time = now
    msg.tmsdb[0].id = 2010
    msg.tmsdb[0].name = "wheelchair_blue"
    msg.tmsdb[0].place = 5001
    msg.tmsdb[0].sensor = 2010
    msg.tmsdb[0].state = 1      # 存在する
    msg.tmsdb[0].x = arg.position.x
    msg.tmsdb[0].y = arg.position.y
    msg.tmsdb[0].z = arg.position.z
    quat = (
        arg.orientation.x,
        arg.orientation.y,
        arg.orientation.z,
        arg.orientation.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    msg.tmsdb[0].rr = roll
    msg.tmsdb[0].rp = pitch
    msg.tmsdb[0].ry = yaw
    db_pub.publish(msg)


rospy.Subscriber("/mimamorukun/dbpose", Pose, callback)
rospy.spin()
# r = rospy.Rate(4)
# while not rospy.is_shutdown():
#     r.sleep()

