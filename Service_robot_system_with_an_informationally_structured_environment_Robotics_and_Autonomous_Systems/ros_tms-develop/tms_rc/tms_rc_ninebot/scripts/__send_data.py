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

#サーバーとポータブルを接続するドライバー
class SendDataClass: #最初は大文字
     """ SendData class ex. サーバーに情報を送る """

     def __init__(self, string, x, y, theta): #コンストラクタ
       self.output_line_ = "Pot," + string + ",plot," + x + "," + y + "," + theta + ",\r"    #アウトプット用文字列
       rospy.Subscriber('send_data', String, self.callback_string, queue_size=10) #オドメトリサブスクライバー

     def callback_string(self,data): #位置情報確認コールバック関数
       self.output_line_ = data.data

def main(args):
  rospy.init_node('sendToserver')
  command_pub  = rospy.Publisher('command_msg', String, queue_size=10)
  sd = SendDataClass(args[1], args[2], args[3], args[4]) #SendDataClass　クラスの宣言
  rate = rospy.Rate(10.0)

  #ソケット通信の設定
  host = '192.168.50.192' #ホストの設定
  port = 4000            #ポートの設定をする
  bufsize = 1024 *2      #バッファサイズの設定
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  with closing(sock):
    sock.connect((host, port))

    #メインループ
    while not rospy.is_shutdown():
      sock.send(sd.output_line_)       #文字列を送る
      command_msg = sock.recv(bufsize) #文字列を受け取る
      command_pub.publish(command_msg)
      rate.sleep()
  return

if __name__ == '__main__':
  main(sys.argv)
