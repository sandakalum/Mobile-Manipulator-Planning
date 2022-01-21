#!/usr/bin/env python
# coding: utf-8

#import module
import serial
import time
import rospy

import tty
import sys
import termios
from std_msgs.msg      import String
from geometry_msgs.msg import Twist
from ninebot.msg       import nucleo_serial

#global
global_cmd = Twist()
ser = serial.Serial()
connect = False

#limit of write value
def limit(value, min_value, max_value):
    return_value = value
    if max_value < return_value:
        return_value = max_value
    if return_value < min_value:
        return_value = min_value
    return return_value

#shutdown
def shutdown():
    global ser
    global connect
    if connect == True:
        try:
            ser.write(chr(127))
            ser.close()
        except: pass
        rospy.loginfo("nucleo_serial : closed serial")

#callback_function
def callback(cmd):
    global global_cmd
    global_cmd = cmd

#advertise_function
def advertise_data():
    
    #ros setting
    global ser
    global global_cmd

    #ros setting
    pub = rospy.Publisher("nucleo_serial", nucleo_serial , queue_size = 1000)
    plot = nucleo_serial()
    L_ser = []

    #serial send
    send_value = 127
    ser.write(chr(send_value))

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            #serial send
            cmd = global_cmd
            limit_value = limit(cmd.angular.z, -5.0, 5.0)
            send_value = int(254 * (limit_value + 5.0) * 0.1)
            ser.write(chr(send_value))
            #rospy.loginfo("nucleo_serial : send_value = %d", send_value)

            #ros publish 
            plot.header.stamp = rospy.Time.now()
            L_ser = ser.readline().split(",")
            if (len(L_ser) < 3): continue
            plot.delta_r = float(L_ser[0])
            plot.delta_l = float(L_ser[1])
            plot.delta_t = float(L_ser[2])
            pub.publish(plot)
            #rospy.loginfo("nucleo_serial : dr = %s dl = %s dt = %s", L_ser[0], L_ser[1], L_ser[2])
        except:
            continue
        rate.sleep()

#main
def main(): 
    
    #global
    global ser
    global connect

    #ros setting
    rospy.Subscriber('mobile_base/commands/velocity', Twist, callback)
    rospy.init_node('portable_driver', anonymous=True)
    rospy.on_shutdown(shutdown)

    #ros param
    port = rospy.get_param('~port', '/dev/ttyACM1')
    baudrate = rospy.get_param('~baud_rate', 115200)

    #serial setting
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    connect = False

    while not rospy.is_shutdown():
        try:
            ser.open()
            time.sleep(5.0)
            connect = True
            break
        except:
            rospy.logerr('Error connecting to Serial : Could not open Serial')
            time.sleep(10.0)
            continue

    #thread
    if connect == True:
        advertise_data()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown()