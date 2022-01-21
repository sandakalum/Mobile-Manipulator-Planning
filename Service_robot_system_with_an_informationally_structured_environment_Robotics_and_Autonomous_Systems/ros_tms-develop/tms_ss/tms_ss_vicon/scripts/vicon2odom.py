#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospay
from geometry_msgs.msg import Odometry
from tms_msg_ss.msg import vicon_data

subjectname = ''
segmentname = ''
pub = rospy.Publisher('odom', Odometry, queue_size=10)


def main():
    print "convet vicon_data to odometry"
    global segmentname, subjectname
    subjectname = rospy.get_param("~subjectname")
    segmentname = rospy.get_param("~segmentname")
    rospy.Subscriber("vicon_data_stream", vicon, callback)
    rospy.spin()
    pass


def callback(vicon):
    if segmentname != vicon.segmentname or subjectname != vicon.subjectname:
        return

    # vicon_data -> Odometry conversion
    pub.publish(odom)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
