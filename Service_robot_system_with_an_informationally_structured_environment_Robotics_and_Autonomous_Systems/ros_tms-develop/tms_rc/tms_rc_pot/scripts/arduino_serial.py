#!/usr/bin/env python
# coding: utf-8

#import module
import serial
import time
import rospy
import math
import thread
import tty
import sys
import termios
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tms_msg_rc.msg import arduino_serial

#global_cmd
global_cmd = Twist() 

#callback_function
def callback(cmd):
    global global_cmd
    global_cmd = cmd

#advertise_function
def advertise_data(p, ser_):
    #ros setting
    global global_cmd
    pub = p.Publisher("arduino_serial", arduino_serial , queue_size = 1000);
    plot = arduino_serial()
    rate = p.Rate(100)
    L_ser = []

    #vel setting
    linear_x   = 0.00 # up
    linear_y   = 0.00 # right
    angular_z  = 0.00 # reverse rotation
    string_ser = "%.2f,%.2f,\0" %(linear_x, angular_z,)

    #timer setting
    now         = rospy.Time.now()
     
    while not p.is_shutdown():
        #time
        now = rospy.Time.now()
        plot.header.stamp = now

        #gloabal -> local
        send_ser = global_cmd

        #serial
        string_ser = "%.2f,%.2f,\0" %(send_ser.linear.x , send_ser.angular.z)     
        ser_.write(string_ser)

        #ros publish
        L_ser = ser_.readline().split(",")
        print L_ser[0], "," , L_ser[1] , "," , L_ser[2],  ",\r"
        
        #ros publish
        plot.encorder_front = long(L_ser[0])
        plot.encorder_left  = long(L_ser[1])
        plot.encorder_back  = long(L_ser[2])
        pub.publish(plot)
        rate.sleep()

#subscribe_command
def subscribe_command(): 
    #ros setting
    rospy.Subscriber("mobile_base/commands/velocity", Twist, callback)
    rospy.init_node('portable_driver', anonymous=True)
    
    #serial setting
    ser = serial.Serial()
    ser.port = '/dev/ttyUSB0'
    ser.baudrate = 115200
    ser.open()
    time.sleep(1.5) #necessary for arudino serial wait time

    #thread
    p = rospy
    thread.start_new_thread(advertise_data, (p, ser))

    #ros spin
    rospy.spin()
   
if __name__ == '__main__':
    try:
         subscribe_command()
    except rospy.ROSInterruptException:
        pass
