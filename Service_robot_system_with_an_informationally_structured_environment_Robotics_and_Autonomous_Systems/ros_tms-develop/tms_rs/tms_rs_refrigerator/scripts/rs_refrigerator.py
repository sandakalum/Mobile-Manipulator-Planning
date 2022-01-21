#!/usr/bin/env python

import time
import datetime
import calendar
import roslib
import rospy
from mbedrpc import*
from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb

mbed = HTTPRPC("192.168.4.239")
startpin = DigitalOut(mbed, p21)
closepin = DigitalOut(mbed, p22)
openpin = DigitalOut(mbed, p24)
clearpin = DigitalOut(mbed, p23)
pub = rospy.Publisher('tms_db_data', TmsdbStamped, queue_size=10)


def control_refrigerator(req):
    msg = TmsdbStamped()
    db = Tmsdb()

    msg.header.stamp = rospy.get_rostime() + rospy.Duration(9 * 60 * 60)

    ms_time = datetime.datetime.fromtimestamp(time.time()).strftime("%Y%m%dT%H%M%S")
    # print ms_time
    db.time = str(ms_time)
    db.id = 2009
    db.sensor = 2009

    if req.service == 1:
        db.state = 2
    elif req.service == 0:
        db.state = 1
    msg.tmsdb.append(db)

    if req.service == 1:
        clearpin.write(0)
        time.sleep(0.1)
        clearpin.write(1)
        time.sleep(0.1)
        startpin.write(0)
        time.sleep(0.1)
        closepin.write(0)
        time.sleep(0.1)
        openpin.write(1)
        time.sleep(0.1)
        startpin.write(1)

        pub.publish(msg)

    elif req.service == 0:
        clearpin.write(0)
        time.sleep(0.1)
        clearpin.write(1)
        time.sleep(0.1)
        startpin.write(0)
        time.sleep(0.1)
        closepin.write(1)
        time.sleep(0.1)
        openpin.write(0)
        time.sleep(0.1)
        startpin.write(1)

        pub.publish(msg)

    else:
        return rs_home_appliancesResponse(0)

    return rs_home_appliancesResponse(1)


def tms_rs_refrigerator():
    rospy.init_node('tms_rs_refrigerator')
    service = rospy.Service('refrigerator_controller', rs_home_appliances, control_refrigerator)
    print "Ready to control the refrigerator"
    rospy.spin()

if __name__ == "__main__":
    tms_rs_refrigerator()
