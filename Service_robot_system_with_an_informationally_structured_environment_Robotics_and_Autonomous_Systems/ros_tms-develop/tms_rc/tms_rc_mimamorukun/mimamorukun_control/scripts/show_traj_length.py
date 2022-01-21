#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""
両車輪の合計移動距離を表示
ロータリーエンコーダの調整に利用
"""

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point

traj = {"l": [Point()], "r": [Point()]}
pose_old = Pose()
base2wheel_l = PointStamped()
base2wheel_l.point.y = -0.27
base2wheel_r = PointStamped()
base2wheel_r.point.y = 0.27


def is_similar(p1, p2):
    # @TODO:角度は無しで，距離の比較しかしていない
    dist = (p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2
    # 距離の差が3cm以下だったらTrue
    return 0.03**2 > dist


def show_length():
    len_l = 0.0
    len_r = 0.0

    def dist(p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return (dx**2 + dy**2)**0.5

    for i in range(len(traj["l"]) - 1):
        len_l += dist(traj["l"][i], traj["l"][i + 1])
        len_r += dist(traj["r"][i], traj["r"][i + 1])
    rospy.loginfo("traj_l:{0}[cm]   traj_r{1}[m] ".format(len_l, len_r))


def callback(input):
    if is_similar(pose_old, input.pose.pose):
        return
    odom2base = TransformStamped()
    odom2base.transform.translation = input.pose.pose.position
    odom2base.transform.rotation = input.pose.pose.orientation
    whl_l_pos = tf2_geometry_msgs.do_transform_point(base2wheel_l, odom2base).point
    traj["l"].append(whl_l_pos)
    whl_r_pos = tf2_geometry_msgs.do_transform_point(base2wheel_r, odom2base).point
    traj["r"].append(whl_r_pos)
    show_length()


def main():
    rospy.init_node('show_traj_length')
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
