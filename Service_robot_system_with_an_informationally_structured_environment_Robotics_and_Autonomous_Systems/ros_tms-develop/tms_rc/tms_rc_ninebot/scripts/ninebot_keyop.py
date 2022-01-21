#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import tf
import math
from geometry_msgs.msg  import PoseStamped
from actionlib_msgs.msg import GoalID
from tms_msg_rc.msg     import nucleo_serial

import subprocess

class Getch:
    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class KeyopClass:

    def __init__(self):
        rospy.init_node('ninebot_keyop', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.goal_frame = '/portable1/base_footprint_combined'
        self.goal_pub   = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.cancel_pub = rospy.Publisher('portable1/move_base/cancel', GoalID, queue_size=10)

    	self.offset_pub = rospy.Publisher('portable1/nucleo_serial', nucleo_serial, queue_size=10)

        offset         = nucleo_serial()
        offset.delta_t = 0.01

        print('\n\nInput key (1->goal_A / 2->goal_B / 3->cancel / f->follower / q->offset+ / e->offset- / ESC->end)\n\n')

	self.now_time = rospy.Time.now()
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():

            getch = Getch()
            x = getch()
            print('\r')

            if  (ord(x) == 27): # 「ESC」
                self.shutdown()
                break

            elif (ord(x) == 3): # 「CTRL + C」
                self.shutdown()
                break

            elif (x == '1'):
                print('set goal position A\n')
                self.goal_plotout(-2.5, 0.0, math.pi)

            elif (x == '2'):
                print('set goal position B\n')
                self.goal_plotout(2.5, 0.0, 0.0)

            elif (x == '3' or x == 'w' or x == 's'):
                print('cancel goal position\n')
                self.cancel_pub.publish(GoalID())

            elif (x == 'f'):
                self.cancel_pub.publish(GoalID())
                print('start ninebot_follower\n')
                subprocess.call("roslaunch tms_rc_ninebot ninebot_follower.launch", shell=True)
                break

            elif (x == 'q' or x == 'a'):
                print('set offset +\n')
                offset.header.stamp = rospy.Time.now()
                offset.raw_delta_r  =  10
                offset.raw_delta_l  = -10
                self.offset_pub.publish(offset)

            elif (x == 'e' or x == 'd'):
                print('set offset -\n')
                offset.header.stamp = rospy.Time.now()
                offset.raw_delta_r  = -10
                offset.raw_delta_l  =  10
                self.offset_pub.publish(offset)

            rate.sleep()

    def goal_plotout(self, goalX, goalY, goalAngle):
        quate = tf.transformations.quaternion_from_euler(0, 0, goalAngle)
        goal = PoseStamped()
        goal.pose.position.x = goalX
        goal.pose.position.y = goalY
        goal.pose.position.z = 0
        goal.pose.orientation.x = quate[0]
        goal.pose.orientation.y = quate[1]
        goal.pose.orientation.z = quate[2]
        goal.pose.orientation.w = quate[3]
        goal.header.frame_id = self.goal_frame
        goal.header.stamp = self.now_time
        self.goal_pub.publish(goal)

    def shutdown(self):
        try:
            self.cancel_pub.publish(GoalID())
        except: pass

if __name__ == '__main__':
    try:
        KeyopClass()
        rospy.loginfo("ninebot_keyop node finished.")
        exit(0)
    except rospy.ROSInterruptException:
        rospy.loginfo("ninebot_keyop node finished.")
