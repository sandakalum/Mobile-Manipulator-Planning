#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import wiringpi2 as pi
import time
from std_msgs.msg import Int32

LINK_UP = 6
LINK_DOWN = 13
HEAD_UP = 19
HEAD_DOWN = 26
FOOT_UP = 12
FOOT_DOWN = 16
HEIGHT_UP = 20
HEIGHT_DOWN = 21

TIME = 20

def stop_all():
    pi.digitalWrite(LINK_UP,1)
    pi.digitalWrite(LINK_DOWN,1)
    pi.digitalWrite(HEAD_UP,1)
    pi.digitalWrite(HEAD_DOWN,1)
    pi.digitalWrite(FOOT_UP,1)
    pi.digitalWrite(FOOT_DOWN,1)
    pi.digitalWrite(HEIGHT_UP,1)
    pi.digitalWrite(HEIGHT_DOWN,1)
    pi.pinMode(LINK_UP,0)
    pi.pinMode(LINK_DOWN,0)
    pi.pinMode(HEAD_UP,0)
    pi.pinMode(HEAD_DOWN,0)
    pi.pinMode(FOOT_UP,0)
    pi.pinMode(FOOT_DOWN,0)
    pi.pinMode(HEIGHT_UP,0)
    pi.pinMode(HEIGHT_DOWN,0)

def move(pin):
    pi.pinMode(pin,1)
    pi.digitalWrite(pin,0)
    time.sleep(TIME)
    pi.digitalWrite(pin,1)
    pi.pinMode(pin,0)

def callback(req):
    rospy.loginfo("move:%d",req.data)
    if req.data == 1:
        move(LINK_UP)
    elif req.data == 2:
        move(LINK_DOWN)
    elif req.data == 3:
        move(HEAD_UP)
    elif req.data == 4:
        move(HEAD_DOWN)
    elif req.data == 5:
        move(FOOT_UP)
    elif req.data == 6:
        move(FOOT_DOWN)
    elif req.data == 7:
        move(HEIGHT_UP)
    elif req.data == 8:
        move(HEIGHT_DOWN)
    else:
        stop_all()

def main():
    pi.wiringPiSetupGpio()
    stop_all()
    rospy.init_node("tms_rc_bed")
    rospy.Subscriber("rc_bed",Int32,callback)
    rospy.loginfo("tms_rc_bed ready")
    rospy.spin()
    rospy.loginfo("exit")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")
        sys.exit(0)
