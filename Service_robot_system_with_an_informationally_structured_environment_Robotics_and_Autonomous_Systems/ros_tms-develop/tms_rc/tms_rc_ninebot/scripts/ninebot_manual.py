#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg  import Twist

import subprocess


ANGLE_CHANGE = 0.01


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

class ManualClass:

    def __init__(self):
        rospy.init_node('ninebot_manual', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.twist_pub = rospy.Publisher('portable1/serial_twist', Twist, queue_size=100)

        key_vel = Twist()
        ang_vel = 0.0

        print('\n\nInput key (a->left / d->right / s->stop / q->end)\n\n')
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():

            getch = Getch()
            x = getch()

            if (x == 'q'):
                self.shutdown()
                break

            elif (ord(x) == 3): # 「CTRL + C」
                self.shutdown()
                break

            elif (x == 'a'):
                ang_vel = ang_vel + ANGLE_CHANGE

            elif (x == 'd'):
                ang_vel = ang_vel - ANGLE_CHANGE

            elif (x == 's'):
                ang_vel = 0.0
            
            if abs(ang_vel) < 0.001 : ang_vel = 0.0 
            
            print('angular_z = ' + str(ang_vel))
            key_vel.angular.z = ang_vel
            self.twist_pub.publish(key_vel)              
            rate.sleep()

    def shutdown(self):
        try:
            self.twist_pub.publish(Twist())
        except: pass

if __name__ == '__main__':
    try:
        ManualClass()
        rospy.loginfo("ninebot_manual node finished.")
        exit(0)
    except rospy.ROSInterruptException:
        rospy.loginfo("ninebot_manual node finished.")
