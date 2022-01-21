#!/usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function
import sys
import socket
import subprocess
import time
from contextlib import closing

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PointStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from tms_msg_ss.msg     import tracking_points
from people_msgs.msg    import People

#ポータブルをtoukatuするドライバー
class ControllerClass: #最初は大文字
     """ SendData class ex. 座標とかレーザレンジファインダの情報を送る """

     def __init__(self, tf, rospy, global_frame, base_footprint, portable_number): #コンストラクタ
       self.output_line_     = ""             #アウトプット用文字列
       self.tf_              = tf             #クラス用tf
       self.back_key_        = False
       self.status_key_      = False
       self.detect_mode_     = False
       self.mainloop_wheel_key_    = False
       self.mainloop_human_key_    = False
       self.rospy_           = rospy          #クラス用rospy
       self.global_frame_    = global_frame   #ポータブル自身のグローバルフレーム
       self.base_footprint_  = base_footprint #ポータブル自身のベースフットプリント
       self.portable_number_ = portable_number #ポータブル自身のナンバー
       self.listener_        = tf.TransformListener() #tfのリスナー
       self.odom_            = Odometry()
       self.twist_           = Twist()
       self.status_          = MoveBaseActionResult()
       self.chairbot_position_ = PointStamped()
       self.human_position_    = tracking_points()
       self.people_position_   = People()
       self.detect_start_command = "roslaunch tms_rc_pot detect_object.launch"
       self.detect_end_command   = "rosnode kill /pot" + portable_number + "/human_tracker /pot" + portable_number + "/reflec_pot"
       self.rate_            = self.rospy_.Rate(10) # 10hz
       self.start_pub_       = self.rospy_.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
       self.goal_pub_        = self.rospy_.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
       self.straight_pub_    = self.rospy_.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
       rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1) #オドメトリサブスクライバー
       rospy.Subscriber('move_base/result', MoveBaseActionResult, self.callback_status, queue_size=1) #リザルトスクライバー
       rospy.Subscriber('lrf_pose', PointStamped, self.callback_wheelchair_position, queue_size=1) #lrf_poseサブスクライバー
       rospy.Subscriber('people', People, self.callback_human_position, queue_size=1) #lrf_poseサブスクライバー
       rospy.Subscriber('command_msg', String, self.callback_command, queue_size=10) #commandサブスクライバー

     def callback_odom(self,data): #位置情報確認コールバック関数
       self.odom_ = data

     def callback_status(self,data): #ステータスコールバック関数
       self.status_ = data
       self.status_key_ = True
       if(self.status_.status.status == 3 and self.back_key_ == True): #目標地点に行った時
         self.rospy_.loginfo("Toback")
         self.go_straight(-0.3) #目標座標に進む前に何メートルか進む(壁から少し離れるため)
         #print("command publish" + self.detect_start_command)
         subprocess.call(self.detect_start_command, shell=True)
         self.detect_mode_ = True

     def callback_command(self, data):
       print(data.data)
       self.select_plot_mode(data.data) #文頭の文字列を読み取ってスタートかゴールを決める

     def get_position(self):  #tfよりポータブル自身の座標を取得する
       (trans,rot) = self.listener_.lookupTransform(self.global_frame_, self.base_footprint_, self.rospy_.Time(0))
       trans_line = ('%.3f' %trans[0]) + ',' + ('%.3f' %trans[1]) + ',' # + ('%.2f' %trans[2]) + ','
       euler = self.tf_.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
       #rot_line = str(rot[0])   + ',' + str(rot[1])   + ',' + str(rot[2])   +  ',' + str(rot[3]) + ','
       self.info_portable_number()
       self.output_line_ = self.output_line_ + 'plot,' + trans_line + ('%.4f' %euler[2]) + ',' #rot_line

     def select_plot_mode(self, string_data): #サーバーから送られてきた文字列の最初でどのモードか選択
       msg_str = string_data.split(",")
       if msg_str[0] == 'start':  #startなら開始目標を出す
         self.start_plot_out(string_data)
       elif msg_str[0] == 'goal': #goalなら目標値を出す
         if self.detect_mode_ == True:
           self.detect_mode_ == False
           #print("command publish" + self.detect_end_command)
           subprocess.call(self.detect_end_command, shell=True)
         self.goal_plot_out(string_data)

     def info_portable_number(self) :#ポータブルのナンバーを知らせる
       self.output_line_ = self.output_line_ + 'Portable,' + self.portable_number_ + ','

     def info_portable_status(self) :#ポータブルのstatusを知らせる
       if self.status_key_ == True:
         self.output_line_ = self.output_line_ + 'status,' + str(self.status_.status.status) + ','

     def add_r_to_output(self) :     #文字列にrを加える関数
       self.output_line_ = self.output_line_ + '\r'

     def clear_output(self) :        #文字列をクリアする関数
       self.output_line_ = ""

     def go_straight(self, distance) :  #目標座標を受け取った時、少しだけまっすぐ進むための関数（距離は指定）
       go_distance = 0.0
       kick_vel = Twist()
       if distance > 0:
         kick_vel.linear.x = 0.15
         self.back_key_ = True
       elif distance < 0:
         kick_vel.linear.x = -0.15
         self.back_key_ = False
       while not rospy.is_shutdown():
         self.straight_pub_.publish(kick_vel)
         go_distance = go_distance + abs(self.odom_.twist.twist.linear.x * 0.1)
         if go_distance > abs(distance):
           break
         #print(str(go_distance) + "go straight")
         self.rate_.sleep()
       kick_vel.linear.x = 0.0
       self.straight_pub_.publish(kick_vel)

     def start_plot_out(self, string_data): #ポータブルの開始座標を投げる関数
       msg_str = string_data.split(",")
       tmp_pose = PoseWithCovarianceStamped()
       num = 0
       while num < 5: #スタート座標を念のため5回投げてあげる
         now         = self.rospy_.Time.now()
         tmp_pose.header.stamp    = now
         tmp_pose.header.frame_id = self.global_frame_
         tmp_pose.pose.pose.position.x = float(msg_str[1]);
         tmp_pose.pose.pose.position.y = float(msg_str[2]);
         tmp_pose.pose.pose.position.z = float(msg_str[3]);
         tmp_pose.pose.pose.orientation.x = float(msg_str[4]);
         tmp_pose.pose.pose.orientation.y = float(msg_str[5]);
         tmp_pose.pose.pose.orientation.z = float(msg_str[6]);
         tmp_pose.pose.pose.orientation.w = float(msg_str[7]);
         self.start_pub_.publish(tmp_pose)
         num += 1

     def goal_plot_out(self, string_data): #ポータブルの目標座標を投げる関数
       msg_str = string_data.split(",")
       if self.back_key_ == False:         #False そもそも目標地点付近なら動く必要がない　
         self.go_straight(0.3)             #目標座標に進む前に何メートルか進む
       time.sleep(8 * (int(self.portable_number_) - 1))
       tmp_pose = PoseStamped()
       num = 0
       while num < 5: #ゴール座標を念のため5回投げてあげる
         now      = self.rospy_.Time.now()
         tmp_pose.header.stamp    = now
         tmp_pose.header.frame_id = self.global_frame_
         tmp_pose.pose.position.x = float(msg_str[1])
         tmp_pose.pose.position.y = float(msg_str[2])
         tmp_pose.pose.position.z = float(msg_str[3])
         tmp_pose.pose.orientation.x = float(msg_str[4])
         tmp_pose.pose.orientation.y = float(msg_str[5])
         tmp_pose.pose.orientation.z = float(msg_str[6])
         tmp_pose.pose.orientation.w = float(msg_str[7])
         self.goal_pub_.publish(tmp_pose)
         num += 1

     def callback_wheelchair_position(self, data): #反射強度から取得できた車いすロボットの座標を取得する
       self.chairbot_position_ = data
       if(self.mainloop_wheel_key_ == True):
         self.mainloop_wheel_key_ = False
         self.output_line_ = self.output_line_ + 'chairbot,' + \
                  ('%.3f' %self.chairbot_position_.point.x) + ',' + \
                  ('%.3f' %self.chairbot_position_.point.y) + ','

     def callback_human_position(self, data):  #人の座標を取得する
       self.people_position_    = data
       if(self.mainloop_human_key_ == True):
         self.mainloop_human_key_ = False
         self.output_line_ = self.output_line_ + 'human,' + str(len(data.people)) + ','
         for i in range(0, len(data.people)):
           self.output_line_ = self.output_line_ + data.people[i].name + ',' +\
                                                   ('%.3f' %data.people[i].position.x) + ',' +\
                                                   ('%.3f' %data.people[i].position.y) + ',' + ('%.3f' %data.people[i].velocity.x) + ',' +\
                                                   ('%.3f' %data.people[i].velocity.y) + ','

def main(args):
  rospy.init_node('sendToserver')
  send_data_pub  = rospy.Publisher('send_data', String, queue_size=10)
  base_footprint  = 'base_footprint'
  portable_number = args[1]
  global_frame    = 'map' + portable_number
  base_footprint = base_footprint + portable_number
  sd = ControllerClass(tf, rospy, global_frame, base_footprint, portable_number) #SendDataClass　クラスの宣言
  rate = rospy.Rate(10.0)

  #メインループ
  while not rospy.is_shutdown():
    try:
       sd.get_position()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       continue
    #portable_number, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w
    sd.info_portable_status()        #ポータブルのステータスを送る
    sd.add_r_to_output()             #文字列の最後処理 rを追加
    send_data_pub.publish(sd.output_line_) #文字列を送る
    sd.clear_output()                #文字列をクリア
    sd.mainloop_wheel_key_    = True
    sd.mainloop_human_key_    = True
    rate.sleep()
  return

if __name__ == '__main__':
  main(sys.argv)
