#!/usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function
import sys
import socket
import subprocess
from subprocess import Popen
import time
import math
from contextlib import closing

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PointStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from tms_msg_ss.msg     import tracking_points
from people_msgs.msg    import People
from std_srvs.srv       import Empty

#ポータブルをtoukatuするドライバー
class ControllerClass: #最初は大文字
     """ SendData class ex. 座標とかレーザレンジファインダの情報を送る """
     def __init__(self, tf, rospy, portable_number): #コンストラクタ
       self.output_line_     = ""             #アウトプット用文字列
       self.tf_              = tf             #クラス用tf
       self.back_key_        = False
       self.status_key_      = False
       self.detect_mode_     = False
       self.mainloop_wheel_key_    = False
       self.mainloop_human_key_    = False
       self.rospy_             = rospy          #クラス用rospy
       self.global_frame_      = 'map' + portable_number   #グローバルフレーム
       self.base_footprint_    = 'base_footprint' + portable_number #ベースフットプリント
       self.portable_number_   = portable_number #ポータブルのナンバー
       self.listener_          = tf.TransformListener() #tfのリスナー
       self.odom_              = Odometry()
       self.twist_             = Twist()
       self.status_            = GoalStatusArray()
       self.chairbot_position_ = PointStamped()
       self.human_position_    = tracking_points()
       self.people_position_   = People()
       self.start_pos_         = PoseWithCovarianceStamped() #potスタートポジ
       self.goal_pos_          = PoseStamped() #potゴールポジ
       self.err_count_         = 0
       self.detect_end_time    =  rospy.Time.from_sec(time.time()).to_sec()
       self.detect_start_command = "roslaunch tms_rc_pot detect_object.launch"
      #  self.detect_end_command   = "rosnode kill /pot"+portable_number+"/human_tracker /pot"+portable_number+"/reflec_pot"
       self.detect_end_command   = "rosnode kill /pot"+portable_number+"/reflec_pot"
       self.rate_            = self.rospy_.Rate(10) # 10hz
       self.start_pub_       = self.rospy_.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
       self.goal_pub_        = self.rospy_.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
       self.vel_pub_         = self.rospy_.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
       rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1) #オドメトリサブスクライバー
       rospy.Subscriber('move_base/status', GoalStatusArray, self.callback_status, queue_size=1) #リザルトスクライバー
       rospy.Subscriber('lrf_pose', PointStamped, self.callback_wheelchair_position, queue_size=1) #lrf_poseサブスクライバー
       rospy.Subscriber('people', People, self.callback_human_position, queue_size=1) #lrf_poseサブスクライバー
       rospy.Subscriber('command_msg', String, self.callback_command, queue_size=10) #commandサブスクライバー
       rospy.wait_for_service('move_base/clear_costmaps')
       self.rect_status      =  rospy.ServiceProxy('move_base/clear_costmaps', Empty)

     def callback_odom(self,data): #位置情報確認コールバック関数 ---
       self.odom_ = data

     def callback_status(self,data): #ステータスコールバック関数(修正いる 4のとき再度目標なげるような)
       #  self.rospy_.loginfo(data)
       self.status_ = data
       if len(self.status_.status_list) > 0:
        self.status_key_ = True
        #now_time = self.rospy_.Time.from_sec(time.time()).to_sec()
        #pass_time = now_time - self.detect_end_time
        #self.rospy_.loginfo("passtime " + str(pass_time))
        self.rospy_.loginfo("status_list " + str(len(self.status_.status_list)))
        if self.status_.status_list[len(self.status_.status_list)-1].status == 3 and self.detect_mode_ == False : #目標地点に行った時
          self.detect_mode_ = True
          self.go_straight(-0.3, 0.1) #目標座標に進む前に何メートルか進む(壁から少し離れるため)
          self.position_calibration()
          proc = Popen(self.detect_start_command, shell=True)
          self.rospy_.loginfo('detect_start_')
          #self.detect_end_time = self.rospy_.Time.from_sec(time.time()).to_sec()
        elif self.status_.status_list[len(self.status_.status_list)-1].status == 4: #ポータブルは動作中自らコストマップに突っ込むことがあるので
          try:
            rec = self.rect_status()
            self.rospy_.logwarn("Clearing costmap. Probably Portable is in collison　" + str(self.err_count_))
            self.goal_plot_out(self.goal_pos_) #再度目標座標を投げる
            self.err_count_ +=1                #エラーのカウントを行う
          except rospy.ServiceException, e:
            self.rospy_.logerr("Clearing costmap service call failed: %s"%e)

     def callback_command(self, data): #文頭の文字列を読み取ってスタートかゴールを決める --
       #self.rospy_.loginfo(data)
       self.select_plot_mode(data.data) #

     def callback_wheelchair_position(self, data): #反射強度から取得できた車いすロボットの座標を取得する　---
       self.chairbot_position_ = data
       if(self.mainloop_wheel_key_ == True):
         self.mainloop_wheel_key_ = False
         self.output_line_ = self.output_line_ + 'chairbot,' + \
                  ('%.3f' %self.chairbot_position_.point.x) + ',' + \
                  ('%.3f' %self.chairbot_position_.point.y) + ','

     def callback_human_position(self, data):  #人の座標を取得する ---
       self.people_position_    = data
       if(self.mainloop_human_key_ == True):
         self.mainloop_human_key_ = False
         self.output_line_ = self.output_line_ + 'human,' + str(len(data.people)) + ','
         for i in range(0, len(data.people)):
           self.output_line_ = self.output_line_ + \
                       data.people[i].name + ',' + \
                      ('%.3f' %data.people[i].position.x) + ',' +\
                      ('%.3f' %data.people[i].position.y) + ',' +\
                      ('%.3f' %data.people[i].velocity.x) + ',' +\
                      ('%.3f' %data.people[i].velocity.y) + ','

     def get_position(self):  #tfよりポータブル自身の座標を取得する ---
       (trans,rot) = self.listener_.lookupTransform(self.global_frame_, self.base_footprint_, self.rospy_.Time(0))
       euler = self.tf_.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
       self.info_portable_number()
       self.output_line_ += 'plot,' +\
                            ('%.2f' %trans[0]) + ',' +\
                            ('%.2f' %trans[1]) + ',' +\
                            ('%.4f' %euler[2]) + ','

     def select_plot_mode(self, string_data): #サーバーから送られてきた文字列の最初でどのモードか選択 ---
       msg_str = string_data.split(",")
       if msg_str[0] == 'start':        #startなら開始目標を出す
         self.start_pos_.header.stamp            = self.rospy_.Time.now()
         self.start_pos_.header.frame_id         = self.global_frame_
         self.start_pos_.pose.pose.position.x    = float(msg_str[1])
         self.start_pos_.pose.pose.position.y    = float(msg_str[2])
         self.start_pos_.pose.pose.position.z    = float(msg_str[3])
         self.start_pos_.pose.pose.orientation.x = float(msg_str[4])
         self.start_pos_.pose.pose.orientation.y = float(msg_str[5])
         self.start_pos_.pose.pose.orientation.z = float(msg_str[6])
         self.start_pos_.pose.pose.orientation.w = float(msg_str[7])
         self.start_plot_out(self.start_pos_)
       elif msg_str[0] == 'goal':       #goalなら目標値を出す
         if self.detect_mode_ == True:  #detect モードなら切る
           self.rospy_.logwarn("kill node human and chairbot tracker")
           proc = Popen(self.detect_end_command, shell=True)
         self.goal_pos_.header.stamp       = self.rospy_.Time.now()
         self.goal_pos_.header.frame_id    = self.global_frame_
         self.goal_pos_.pose.position.x    = float(msg_str[1])
         self.goal_pos_.pose.position.y    = float(msg_str[2])
         self.goal_pos_.pose.position.z    = float(msg_str[3])
         self.goal_pos_.pose.orientation.x = float(msg_str[4])
         self.goal_pos_.pose.orientation.y = float(msg_str[5])
         self.goal_pos_.pose.orientation.z = float(msg_str[6])
         self.goal_pos_.pose.orientation.w = float(msg_str[7])
         self.goal_plot_out(self.goal_pos_)
         self.detect_mode_ = False
       elif msg_str[0] == 'detect':     #ディテクトならキャリブレーションをしたのち人物＆車いす追跡ノードを立ち上げる
         self.start_pos_.header.stamp            = self.rospy_.Time.now()
         self.start_pos_.header.frame_id         = self.global_frame_
         self.start_pos_.pose.pose.position.x    = float(msg_str[1])
         self.start_pos_.pose.pose.position.y    = float(msg_str[2])
         self.start_pos_.pose.pose.position.z    = float(msg_str[3])
         self.start_pos_.pose.pose.orientation.x = float(msg_str[4])
         self.start_pos_.pose.pose.orientation.y = float(msg_str[5])
         self.start_pos_.pose.pose.orientation.z = float(msg_str[6])
         self.start_pos_.pose.pose.orientation.w = float(msg_str[7])
         self.start_plot_out(self.start_pos_)
         self.detect_mode_ = True
         self.position_calibration()    #位置キャリブレーション
         subprocess.call(self.detect_start_command, shell=True) #人物＆車いす追跡ノードの立ち上げ

     def info_portable_number(self):#ポータブルのナンバーを知らせる ---
       self.output_line_ = self.output_line_ + 'Pot,' + self.portable_number_ + ','

     def info_portable_status(self):#ポータブルのstatusを知らせる ---
       if self.status_key_ == True:
         self.output_line_ = self.output_line_ + 'status,' + str(self.status_.status_list[0].status) + ','

     def add_r_to_output(self):     #文字列にrを加える関数 ---
       self.output_line_ = self.output_line_ + '\r'

     def clear_output(self):        #文字列をクリアする関数 ---
       self.output_line_ = ""

     def go_straight(self, distance, linear):  #ポータブルを指定させた距離だけ直進させる関数　---
       go_distance = 0.0
       kick_vel = Twist()
       if distance > 0:
         kick_vel.linear.x = linear
       elif distance < 0:
         kick_vel.linear.x = -(linear)
       while not rospy.is_shutdown():
         self.vel_pub_.publish(kick_vel)
         go_distance = go_distance + abs(self.odom_.twist.twist.linear.x * 0.1)
         if go_distance > abs(distance):
           break
         self.rate_.sleep()
       kick_vel.linear.x = 0.0
       self.vel_pub_.publish(kick_vel)

     def turn_arround(self, angle, angular):  #ポータブルをで指定した角度分回転させる関数 ---
       count = 0
       if angle > math.pi:
         while angle > math.pi:
           angle -= math.pi
           count +=1
       elif angle < math.pi:
         while angle < -(math.pi):
           angle += math.pi
           count +=1
       kick_vel = Twist()
       sb_angle = math.pi
       while True:
         if count == 0:
           sb_angle = angle
         if angle > 0:
           kick_vel.angular.z = angular
         if angle < 0:
           kick_vel.angular.z = -(angular)
         turn_angle = 0.0
         while not rospy.is_shutdown():
           self.vel_pub_.publish(kick_vel)
           turn_angle += abs(self.odom_.twist.twist.angular.z * 0.1)
           if turn_angle > abs(sb_angle):
             break
           self.rate_.sleep()
         kick_vel.angular.z = 0.0
         self.vel_pub_.publish(kick_vel)
         if count == 0:
           break
         count -=1

     def position_calibration(self): #ポジションキャリブレーション ---
       self.turn_arround(2 * math.pi, 0.8)
       self.go_straight(0.15, 0.07)
       self.go_straight(-0.15, 0.07)

     def start_plot_out(self, start): #ポータブルの開始座標を投げる関数 ---
       num = 0
       while num < 5: #スタート座標を念のため5回投げてあげる
         self.start_pub_.publish(start)
         num += 1

     def goal_plot_out(self, goal): #ポータブルの目標座標を投げる関数 ---
       self.go_straight(0.3, 0.1)                      #目標座標に進む前に何メートルか進む
       #time.sleep(8 * (int(self.portable_number_) - 1)) #時間を最初に時間を置く　最初だけ
       num = 0
       while num < 5: #ゴール座標を念のため5回投げてあげる
         self.goal_pub_.publish(goal)
         num += 1

#メイン関数
def main(args):
  rospy.init_node('sendToserver')
  send_data_pub  = rospy.Publisher('send_data', String, queue_size=10)
  portable_number = args[1]
  sd = ControllerClass(tf, rospy, portable_number) #SendDataClass　クラスの宣言
  rate = rospy.Rate(10.0)

  #メインループ
  while not rospy.is_shutdown():
    try:
       sd.get_position()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       continue
    #portable_number, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w
    sd.info_portable_status()              #ポータブルのステータスを送る
    sd.add_r_to_output()                   #文字列の最後処理 rを追加
    send_data_pub.publish(sd.output_line_) #文字列を送る
    sd.clear_output()                      #文字列をクリア
    sd.mainloop_wheel_key_    = True       #ここらへんもまとめたい
    sd.mainloop_human_key_    = True
    rate.sleep()
  return

#ポータブルコントローラ
if __name__ == '__main__':
  main(sys.argv)
