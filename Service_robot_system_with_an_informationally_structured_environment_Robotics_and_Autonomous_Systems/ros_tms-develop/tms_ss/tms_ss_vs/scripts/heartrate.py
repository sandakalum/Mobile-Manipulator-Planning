#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import json
import datetime
import time
import subprocess
import rospy
# import roslib
from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb

DEV_PORT = "/dev/ttyUSB0"

if __name__ == '__main__':
    print "Hello World"

    # init device
    cmd_chmod = "sudo chmod a+rw " + DEV_PORT
    print cmd_chmod + "\n", subprocess.check_output(cmd_chmod.split(" "))
    dev = serial.Serial(port=DEV_PORT, baudrate=9600)

    # init ROS
    rospy.init_node('tms_ss_vs_heartrate')
    db_pub = rospy.Publisher('tms_db_data', TmsdbStamped, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()
        # get heartrate value
        dev.write("G5\r")  # get heartrate buffer[0~4]
        time.sleep(0.5)
        ret = dev.read(dev.inWaiting())
        rate = ret.split(" ")[2]
        print "heartrate:" + rate + "     raw:" + ret
        if int(rate) == 0:
            print "    failed to get heartrate value"
            continue

        # make json text
        note_d = {"heartrate": rate}
        # print json.dumps(note_d, indent=4)
        note_j = json.dumps(note_d)

        # regist to DB
        msg = TmsdbStamped()
        db = Tmsdb()
        db.time = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
        db.name = "heartrate_sensor"
        db.id = 3018
        db.sensor = 3018
        db.place = 1001
        db.note = note_j
        msg.tmsdb.append(db)
        msg.header.stamp = rospy.get_rostime() + rospy.Duration(9 * 60 * 60)
        db_pub.publish(msg)
