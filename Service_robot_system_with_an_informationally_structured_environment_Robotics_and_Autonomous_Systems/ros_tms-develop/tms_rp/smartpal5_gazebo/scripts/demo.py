#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import random

import os
import sys


def demo():
    rospy.init_node('demo')
    pub = rospy.Publisher("joint_angle_smartpal", Float32MultiArray, queue_size=1)

    dim = [MultiArrayDimension(label='Dimension1', size=1, stride=8)]
    data_offset = 1
    layout = MultiArrayLayout(dim, data_offset)
    data1 = [0, 0.08, 0, 1.57, 0, 0, 0, -1.0]
    data2 = [1, 0.08, 0, 0.5, 0, 0, 0, -1.0]

    while not rospy.is_shutdown():
        topic = Float32MultiArray(layout, data2)
        pub.publish(topic)
        rospy.sleep(0.1)
    print 'done'

if __name__ == '__main__':
    try:
        demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("demo node terminated.")
