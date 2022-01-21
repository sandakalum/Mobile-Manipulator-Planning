#!/usr/bin/env python
# -*- coding:utf-8 -*-

# move_baseから速度指令(cmd_vel)を受け取って、最終的にNinebotへ送る指令(serial_twist)へ変換する
# その際にTwist型の「linear.z」を音を鳴らす指令として用いている
# liniar.zが0以外の時に警告音を鳴らし、0の時に音は停止する

import rospy
import tf
import math
import actionlib

from actionlib_msgs.msg import *
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

ANGULAR_MAX  = 0.2  # 最終的に出力される角速度の絶対値の最大値
ANGULAR_K    = 0.5  # 曲率を維持する際のスケール変更の係数(マジックナンバー) 

ACCELE_CW    = 0.03 # 台形加速の角加速度(100Hzの１ループ中での角速度の最大増減分)
#LOW_ALPHA    = 0.9  # 台形加速のためにローパスフィルタを使用する際のアルファ


class SmootherClass:
  def __init__(self):

    rospy.init_node('ninebot_smoother', anonymous=True)
    rospy.on_shutdown(self.shutdown)

    self.now_twist    = Twist()
    self.serial_twist = Twist()
    self.twist_pub    = rospy.Publisher('serial_twist', Twist, queue_size=100)

    rospy.Subscriber('cmd_vel', Twist, self.callback_twist, queue_size=10)
    rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=10)
    rospy.Subscriber('move_base/status', GoalStatusArray, self.callback_status, queue_size=10)

    self.status_id = 0
    self.count = 0

    p_tgt_vw = 0.0
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

      serial_twist = self.serial_twist
      tgt_vw = serial_twist.angular.z
      
      #最大角加速度にしたがって、前回値から角速度を更新する（静止命令の時は角加速度は無視して静止する）
      if   ((tgt_vw > 0.0) and (tgt_vw > p_tgt_vw + ACCELE_CW)): tgt_vw = p_tgt_vw + ACCELE_CW
      elif ((tgt_vw < 0.0) and (tgt_vw < p_tgt_vw - ACCELE_CW)): tgt_vw = p_tgt_vw - ACCELE_CW
      #tgt_vw = p_tgt_vw * LOW_ALPHA +  tgt_vw * (1.0 - LOW_ALPHA)
      p_tgt_vw = tgt_vw

      serial_twist.angular.z = self.limit(tgt_vw, -ANGULAR_MAX, ANGULAR_MAX)
      self.twist_pub.publish(serial_twist)
      rate.sleep()

  def callback_twist(self, data):
    now_twist = self.now_twist
    
    #角速度が小さい時に摩擦に打ち勝てるように増幅させるように変換
    # if   data.angular.z > 0: data.angular.z =  math.sqrt(10 * math.fabs(data.angular.z)) * 0.31
    # elif data.angular.z < 0: data.angular.z = -math.sqrt(10 * math.fabs(data.angular.z)) * 0.31

    #想定の方向への速度が一定速以上出ている場合は、なるべく曲率を維持した角速度を計算する
    #if (math.fabs(now_twist.linear.x) > 0.08) and (now_twist.linear.x * data.linear.x > 0.0):
    #  data.angular.z = data.angular.z * ANGULAR_K * math.fabs(now_twist.linear.x / data.linear.x)
 
    #速度が一定速以上出ていて、バックする方向へ指令が出ている場合は、音を鳴らして警告する
    #if (math.fabs(now_twist.linear.x) > 0.08) and (data.linear.x < 0.0):
    if data.linear.x < 0.0:
      data.linear.z = 1.0

    #ゴールでは音を出す
    #if self.status_id == 3:
    #  data.linear.z = 1.0

    self.serial_twist = data

  def callback_odom(self, data):
    self.now_twist = data.twist.twist

  def callback_status(self, data):
    # PENDING         = 0  
    # ACTIVE          = 1 
    # PREEMPTED       = 2
    # SUCCEEDED       = 3
    # ABORTED         = 4
    # REJECTED        = 5
    # PREEMPTING      = 6
    # RECALLING       = 7
    # RECALLED        = 8
    # LOST            = 9

    if len(data.status_list) > 0:
       goalStatus = data.status_list[0];
       self.status_id = goalStatus.status;
       #print(self.status_id)

    #ゴールでは音を出す
    if self.status_id == 3:
       self.count = self.count + 1
       if self.count < 5:
          self.serial_twist.linear.z = 1.0
       else:
          self.serial_twist.linear.z = 0.0
    else:
       self.count = 0

  def limit(self, value, min_value, max_value):
    return_value = value
    if max_value < return_value:
      return_value = max_value
    if return_value < min_value:
      return_value = min_value
    return return_value

  def shutdown(self):
    try:
      self.twist_pub.publish(Twist())
    except: pass

if __name__ == '__main__':
  try:
    SmootherClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_smoother node finished.")
