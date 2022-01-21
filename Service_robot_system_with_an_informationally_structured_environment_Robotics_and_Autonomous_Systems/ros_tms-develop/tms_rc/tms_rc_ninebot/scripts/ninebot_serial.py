#!/usr/bin/env python
# -*- coding:utf-8 -*-

#--低レベルとの通信規格
#　  全て１バイト単位で通信（１バイトを一定時間受け取れなかった場合低レベル側で停止命令が出る）
#　  上位１ビットが1の時警告ブザーを鳴らし、0の時は音は停止
#　  下位7ビットが0~126の時、63を中心とし63より小さい時はCW方向、大きい時はCCW方向へ回転
#    127の時は静止（１２７は緊急停止として用意しているが現在のところ63と127の違いはなし）
#    0の時:中心値 - 0.2[v] 、 126の時:中心値 + 0.2[V]が回路からNinebotへ出される
#    （低レベル側では、中心値±0.2[V]までしか出ないように設定している）

import serial
import time
import rospy

import tty
import sys
import termios
from std_msgs.msg      import String
from geometry_msgs.msg import Twist
from tms_msg_rc.msg    import nucleo_serial


ANGULAR_MAX  = 0.2  # ninebot_smootherから受信する命令の角速度の絶対値の最大値
VOLTAGE_MAX  = 0.13 # Ninebotへは(cmd.angular.z = ANGULAR_MAXの時に)中心値±VOLTAGE_MAX[V]送信される (MAX:0.2)

K_SEND_VALUE = VOLTAGE_MAX / 0.2


class SerialClass:
  def __init__(self):
    rospy.init_node('ninebot_serial', anonymous=True)
    rospy.on_shutdown(self.shutdown)

    self.cmd          = Twist()
    self.ser          = serial.Serial()
    self.ser.port     = rospy.get_param('~port', '/dev/ttyACM1')
    self.ser.baudrate = rospy.get_param('~baud_rate', 115200)
       
    while True:
      try:
        self.ser.open()
        time.sleep(5)
        break
      except:
        rospy.logerr('Error connecting to Serial : Could not open Serial')
        time.sleep(5)
        continue 

    rospy.Subscriber('serial_twist', Twist, self.callback, queue_size=100)
    
    plot     = nucleo_serial()
    pub      = rospy.Publisher("nucleo_serial", nucleo_serial , queue_size=1000)
    line_ser = []

    send_value = 127
    self.ser.write(chr(send_value))

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      rate.sleep()
      try:
        ang_value = self.limit(self.cmd.angular.z, -ANGULAR_MAX, ANGULAR_MAX) #-ANGULAR_MAX ≦ ang_value ≦ ANGULAR_MAX
        ang_value = (ang_value + ANGULAR_MAX) / (2 * ANGULAR_MAX)             #           0 ≦ ang_value ≦ 1
        ang_value = int(126 * (0.5 + ((ang_value - 0.5) * K_SEND_VALUE)))     #           0 ≦ ang_value ≦ 126

        if self.cmd.linear.z != 0.0: send_value = 0x80 | ang_value 
        else                       : send_value = 0x7f & ang_value  
        
        self.ser.write(chr(send_value))
        #rospy.loginfo("nucleo_serial : send_value = %d", send_value)

        line_ser = self.ser.readline().split(",")
        if (len(line_ser) < 3): continue
        plot.header.stamp = rospy.Time.now()
        plot.raw_delta_r = int(line_ser[0])
        plot.raw_delta_l = int(line_ser[1])
        plot.delta_t   = float(line_ser[2])
        pub.publish(plot)
        #rospy.loginfo("nucleo_serial : dr = %s dl = %s dt = %s", line_ser[0], line_ser[1], line_ser[2])
      
      except: continue

  def callback(self, data):
    self.cmd = data
  
  def limit(self, value, min_value, max_value):
    return_value = value
    if max_value < return_value:
      return_value = max_value
    if return_value < min_value:
      return_value = min_value
    return return_value

  def shutdown(self):
    try:
      self.ser.write(chr(127))
      self.ser.close()
    except: pass

if __name__ == '__main__':
  try:
    if   K_SEND_VALUE > 1: K_SEND_VALUE = 1
    elif K_SEND_VALUE < 0: K_SEND_VALUE = 0
    SerialClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_serial node finished.")
