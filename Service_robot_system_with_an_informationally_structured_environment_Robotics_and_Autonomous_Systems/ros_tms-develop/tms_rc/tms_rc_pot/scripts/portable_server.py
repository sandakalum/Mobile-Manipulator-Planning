#!/usr/bin/env python
# coding: utf-8
from __future__ import print_function
import socket
import select
import rospy
import tf
import math
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg      import String
from people_msgs.msg   import People
from people_msgs.msg   import Person
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class PersonInfoClass:
     """ SendDaself.ta class ex. 人のクラス """
     def __init__(self): #コントラクタ
       self.id = 0
       self.x  = 0.0
       self.y  = 0.0
       self.x_vel = 0.0
       self.y_vel = 0.0

class PortablePlotClass:
     """ SendDaself.ta class ex. ポータブルの各座標を収納するためのクラス """
     def __init__(self,rospy): #コンストラクタ
       self.send_data_    = "" #potアウトプット用
       self.number_       = 0
       self.start_pos_    = PoseWithCovarianceStamped() #potスタートポジ
       self.goal_pos_     = PoseStamped() #potゴールポジ
       self.detect_pos_   = PoseStamped() #ディテクティブポジ
       self.now_point_    = PoseStamped() #ナウポジション
       self.key           = 0 #サブスクライブが起きたか判定
       self.register_key_ = False #登録があったか起きたか判定
       self.goal_register_key_   = False #目標値の登録が合った時にTrue
       self.detect_register_key_ = False #detect位置の登録が合った時にTrue
       self.stamp_        = rospy.Time.now() #スタンプ　最終ログイン

class PortableServerClass: #最初は大文字
     """ SendDaself.ta class ex. ポータブルのサーバークラス """
     def __init__(self, rospy, tf): #コンストラクタ
       self.rospy_ = rospy
       self.tf_    = tf
       self.call_back_key = 0 #no commandを送る
       self.pot_class  = []
       self.human_class = []
       self.msg_command = ""
       self.rate_key_   = False
       self.register_portable_ = 0
       self.register_portable_counter_ = 0
       self.chairbot_pub_      = self.rospy_.Publisher('lrf_pose', PointStamped, queue_size=10)
       self.chairbot_odom_pub_ = self.rospy_.Publisher('lrf_pose_odom', Odometry, queue_size=10)
       self.human_pub_         = self.rospy_.Publisher('people', People, queue_size=10)
       self.human_vis_pub_     = self.rospy_.Publisher('people_vis', MarkerArray, queue_size=10)
       self.pot_cloud_         = self.rospy_.Publisher('pot_cloud', PointCloud, queue_size=10)
       self.target_pub_        = self.rospy_.Publisher('target_vis', MarkerArray, queue_size=10)
       self.prev_ids = []
       rospy.Subscriber('start_position', PoseWithCovarianceStamped, self.start_callback, queue_size=10)
       rospy.Subscriber('goal_position',PoseStamped,self.goal_callback, queue_size=10)
       rospy.Subscriber('detect_position',PoseStamped,self.detect_callback, queue_size=10)
       for i in range(1, 13): #数合わせのため11個ポータブルのクラスを作る
         self.pot_class.append(PortablePlotClass(rospy))

     def start_callback(self, data): #スタートコールバック関数
       self.pot_class[int(data.header.frame_id)].number_    = int(data.header.frame_id)
       self.pot_class[int(data.header.frame_id)].start_pos_ = data
       self.pot_class[int(data.header.frame_id)].send_data_ = \
       'start,' + str(data.pose.pose.position.x) + ',' + \
       str(data.pose.pose.position.y)    + ',' + str(data.pose.pose.position.z) + ',' + \
       str(data.pose.pose.orientation.x) + ',' + str(data.pose.pose.orientation.y) + ',' + \
       str(data.pose.pose.orientation.z) + ',' + str(data.pose.pose.orientation.w)  + ','
       self.pot_class[int(data.header.frame_id)].key = 1 #1のときスタートコールバック関数が起きたとする

     def goal_callback(self, data): #ゴールコールバック関数
       self.pot_class[int(data.header.frame_id)].goal_register_key_ = True
       self.pot_class[int(data.header.frame_id)].number_    = int(data.header.frame_id)
       self.pot_class[int(data.header.frame_id)].goal_pos_  = data
       self.pot_class[int(data.header.frame_id)].send_data_ = \
       'goal,' +  str(data.pose.position.x) + ',' + \
       str(data.pose.position.y) + ',' + str(data.pose.position.z) + ',' + \
       str(data.pose.orientation.x) + ',' + str(data.pose.orientation.y) + ',' + \
       str(data.pose.orientation.z) + ',' + str(data.pose.orientation.w)  + ','
       self.pot_class[int(data.header.frame_id)].key = 2 #2のときゴールバック関数が起きたとする

     def detect_callback(self, data): #ディティクトコールバック関数
       self.pot_class[int(data.header.frame_id)].detect_register_key_ = True
       self.pot_class[int(data.header.frame_id)].number_     = int(data.header.frame_id)
       self.pot_class[int(data.header.frame_id)].detect_pos_ = data
       self.pot_class[int(data.header.frame_id)].send_data_ = \
       'detect,' +  str(data.pose.position.x) + ',' + \
       str(data.pose.position.y) + ',' + str(data.pose.position.z) + ',' + \
       str(data.pose.orientation.x) + ',' + str(data.pose.orientation.y) + ',' + \
       str(data.pose.orientation.z) + ',' + str(data.pose.orientation.w)  + ','
       self.pot_class[int(data.header.frame_id)].key = 3 #3のときディテクトコールバック関数が起きたとする

     def string_analysis(self, data): #文字列を解析する関数
       self.rospy_.loginfo(data)
       data_split = data.split(",")
       list_number_portable = data_split.index("Pot")
       list_number_plot     = data_split.index("plot")
       num   = data_split[list_number_portable + 1]      #ポータブルの番号
       x     = float(data_split[list_number_plot + 1])   #x
       y     = float(data_split[list_number_plot + 2])   #y
       theta = float(data_split[list_number_plot + 3])   #theta
       self.register_function(num)
       self.send_command_analysis(num)
       self.publish_portable_position(num, x, y, theta)
       try: #車いすの情報をパブリッシュ
         list_number_chair    = data_split.index("chairbot")     #クルマイスの情報
         chair_x   = float(data_split[list_number_chair + 1])    #クルマイスの x
         chair_y   = float(data_split[list_number_chair + 2])    #クルマイスの y
         self.publish_chairbot_position(num, chair_x, chair_y)
       except ValueError:
         self.rospy_.loginfo("chairbot_out")

       try: #人の情報をパブリッシュ
         list_number_human    = data_split.index("human")   #人の情報
         human_num = int(data_split[list_number_human + 1]) #人の数
         for i in range(0, human_num):                      #人の数だけクラスを作る
           tmp_person_info       = PersonInfoClass()
           tmp_person_info.id    = int(data_split[list_number_human + 2 + i*5])
           tmp_person_info.x     = float(data_split[list_number_human + 3 + i*5])
           tmp_person_info.y     = float(data_split[list_number_human + 4 + i*5])
           tmp_person_info.x_vel = float(data_split[list_number_human + 5 + i*5])
           tmp_person_info.y_vel = float(data_split[list_number_human + 6 + i*5])
           self.human_position_filter(tmp_person_info) #距離が近いものは消し、そうでないものは残す
           #self.human_class.append(PersonInfoClass())
           #self.human_class[i].id = int(data_split[list_number_human + 2 + i*5])
           #self.human_class[i].x     = float(data_split[list_number_human + 3 + i*5])
           #self.human_class[i].y     = float(data_split[list_number_human + 4 + i*5])
           #self.human_class[i].x_vel = float(data_split[list_number_human + 5 + i*5])
           #self.human_class[i].y_vel = float(data_split[list_number_human + 6 + i*5])
         self.register_portable_counter_ = self.register_portable_counter_ + 1
         if(self.register_portable_counter_ > 2 ):#self.register_portable_
           self.rospy_.loginfo("register_portable_counter_ " + str(self.register_portable_counter_))
           self.rospy_.loginfo("register_portable_ " + str(self.register_portable_))
           self.rospy_.loginfo("human_class " + str(len(self.human_class)))
           self.publish_human_position()
           self.human_class = []
           self.register_portable_counter_ = 0
       except ValueError:
         self.rospy_.loginfo("human_out")

     def register_function(self, pot_num): #ポータブルの登録を行う&登録されたポータブルのカウントを行う
       if self.pot_class[int(pot_num)].register_key_ == False:
          self.pot_class[int(pot_num)].register_key_ = True
          self.register_portable_ += 1
          self.rate_key_ = True
       self.pot_class[int(pot_num)].stamp_ = self.rospy_.Time.now()

     def send_command_analysis(self, pot_num): #ポータブル側にコマンドを送る  now goal start どれかを送る
       if self.pot_class[int(pot_num)].key == 0:     #0のときはno command
         self.msg_command = 'no command'
       else :
         self.msg_command = self.pot_class[int(pot_num)].send_data_
       self.pot_class[int(pot_num)].key = 0

     def publish_portable_position(self, num_, x_, y_, theta_): #ポータブルの座標を格納 & 出力
       self.pot_class[int(num_)].now_point_.pose.position.x = x_
       self.pot_class[int(num_)].now_point_.pose.position.y = y_
       self.pot_class[int(num_)].now_point_.pose.position.z = 0.0
       br   = self.tf_.TransformBroadcaster()
       quat = self.tf_.transformations.quaternion_from_euler(0.0, 0.0, theta_)
       br.sendTransform((x_, y_, 0.0), (quat[0],quat[1],quat[2],quat[3]),
                                     self.rospy_.Time.now(),'pot_' + num_,'map')
       br2   = self.tf_.TransformBroadcaster()
       br2.sendTransform((0.03,0.0,0.16),(0.0,0.0,0.0,1.0),self.rospy_.Time.now(),'pot_laser_' + num_,'pot_' + num_)

     def publish_chairbot_position(self,num_, x_, y_,): #クルマイスの座標を送る
       p = PointStamped()
       p.header.stamp    = self.rospy_.Time.now()
       p.header.frame_id = 'pot_laser_'+num_
       p.point.x = x_
       p.point.y = y_
       p.point.z = 0.0

       p_odom = Odometry()
       p_odom.header.stamp         = self.rospy_.Time.now()
       p_odom.header.frame_id      = 'pot_laser_'+num_
       p_odom.pose.pose.position.x = x_
       p_odom.pose.pose.position.y = y_
       p_odom.pose.pose.position.z = 0.30
       quat = self.tf_.transformations.quaternion_from_euler(0.0, 3.1415/2.0, 0.0)
       p_odom.pose.pose.orientation.x = quat[0]
       p_odom.pose.pose.orientation.y = quat[1]
       p_odom.pose.pose.orientation.z = quat[2]
       p_odom.pose.pose.orientation.w = quat[3]
       self.chairbot_pub_.publish(p)
       self.chairbot_odom_pub_.publish(p_odom)

     def human_position_filter(self, tmp_human_info): #人の座標を統括し、filterにかける 距離が近いものは消し、そうでないものは残す
       if len(self.human_class) == 0: #リストに人間の座標が入っていなかった場合，いれる
         self.human_class.append(tmp_human_info)
       else:
         for i in range(0, len(self.human_class)):#人間の座標が入っていた場合，１ｍ以下のものを消し、tmp_human_infoを追加
          self.rospy_.loginfo("self.human_class[i].x " + str(self.human_class[i].x) + " tmp_human_info.x" + str(tmp_human_info.x))
          self.rospy_.loginfo("self.human_class[i].y " + str(self.human_class[i].y) + " tmp_human_info.y" + str(tmp_human_info.y))
          dist = math.pow((self.human_class[i].x - tmp_human_info.x),2)+pow((self.human_class[i].y - tmp_human_info.y),2)
          self.rospy_.loginfo("dist        " + str(dist))
          if dist < pow(1.5, 2): #1.5m以下なら入れない
            break
          if i == len(self.human_class)-1: #リストの最後までなかったら
            self.human_class.append(tmp_human_info)
     def publish_human_position(self): #人の座標を出力
       obstacle_people = People() #people_msg
       obstacle_people.header.frame_id = 'map'
       obstacle_people.header.stamp = self.rospy_.Time.now()
       marker_array_vis = MarkerArray() #people_visalizaion

       ids = map(lambda x:x.id, self.human_class)
       vanished_ids = list(set(self.prev_ids) - set(ids))

       self.prev_ids = map(lambda x:x.id, self.human_class)
       for i in vanished_ids:
      #  xrange(10000):
          marker_vis       = Marker() #marker
          marker_vis.header.frame_id = 'map'
          marker_vis.header.stamp = self.rospy_.Time.now()
          marker_vis.id           = i
          marker_vis.type         = 3 #3 cylinder 9 text
          marker_vis.action       = 2 #delete
          marker_array_vis.markers.append(marker_vis)
       self.human_vis_pub_.publish(marker_array_vis)

       marker_array_vis = MarkerArray() #people_visalizaion
       for i in range(0, len(self.human_class)):
         tmp_human = Person() #person
         tmp_human.name = str(self.human_class[i].id)
         tmp_human.position.x  = self.human_class[i].x
         tmp_human.position.y  = self.human_class[i].y
         tmp_human.position.z  = 0.0
         tmp_human.velocity.x  = self.human_class[i].x_vel
         tmp_human.velocity.y  = self.human_class[i].y_vel
         tmp_human.velocity.z  = 0.0
         tmp_human.reliability = 1.0;
         obstacle_people.people.append(tmp_human)

         marker_vis       = Marker() #marker
         marker_vis.header.frame_id = 'map'
         marker_vis.header.stamp = self.rospy_.Time.now()
         marker_vis.id           = self.human_class[i].id
         marker_vis.type         = 3 #3 cylinder 9 text
         marker_vis.action       = 0 #add
         marker_vis.scale.x      = 0.1
         marker_vis.scale.y      = 0.1
         marker_vis.scale.z      = 1.0
         marker_vis.color.r = 1
         marker_vis.color.g = 0
         marker_vis.color.b = 0
         marker_vis.color.a = 1.0
         marker_vis.lifetime = self.rospy_.Duration(0.2)
         marker_vis.pose.position.x = self.human_class[i].x
         marker_vis.pose.position.y = self.human_class[i].y
         marker_vis.pose.position.z = 0.5
         marker_vis.pose.orientation.x = 0.0
         marker_vis.pose.orientation.y = 0.0
         marker_vis.pose.orientation.z = 0.0
         marker_vis.pose.orientation.w = 1.0
         #marker_vis.text="aaaaaaaaa"
         marker_array_vis.markers.append(marker_vis)
       self.human_pub_.publish(obstacle_people)
       self.human_vis_pub_.publish(marker_array_vis)

     def publish_portable_cloud_point(self): #ポータブルの座標のポイントクラウドをパブリッシュ 後はモデルでもパブリッシュすればいいかな
       portable_cloud = PointCloud()
       portable_cloud.header.stamp = self.rospy_.Time.now()
       portable_cloud.header.frame_id = 'map'
       for i in range(1, 12): #数合わせのため11個ポータブルのクラスを作る
         if self.pot_class[i].register_key_ == True:
           tmp_point      = Point32()
           tmp_point.x = self.pot_class[i].now_point_.pose.position.x;
           tmp_point.y = self.pot_class[i].now_point_.pose.position.y;
           tmp_point.z = 0.0;
           portable_cloud.points.append(tmp_point)
       self.pot_cloud_.publish(portable_cloud)

     def publish_target_point(self): #ポータブルの目標座標を画面表示する
       marker_array_vis = MarkerArray() #people_visalizaion
       for i in range(1, 12):
         if self.pot_class[i].goal_register_key_ == True:
           marker_vis       = Marker() #marker
           marker_vis.header.frame_id = 'map'
           marker_vis.header.stamp = self.rospy_.Time.now()
           marker_vis.id           = i
           marker_vis.type         = 3 #cylinder
           marker_vis.action       = 0 #add
           marker_vis.scale.x      = 0.2
           marker_vis.scale.y      = 0.2
           marker_vis.scale.z      = 1.0
           marker_vis.color.r = 1
           marker_vis.color.g = 1
           marker_vis.color.b = 1
           marker_vis.color.a = 1.0
           marker_vis.lifetime = self.rospy_.Duration(0.5)
           marker_vis.pose.position.x = self.pot_class[i].goal_pos_.pose.position.x
           marker_vis.pose.position.y = self.pot_class[i].goal_pos_.pose.position.y
           marker_vis.pose.position.z = 0.3
           marker_vis.pose.orientation.x = 0.0
           marker_vis.pose.orientation.y = 0.0
           marker_vis.pose.orientation.z = 0.0
           marker_vis.pose.orientation.w = 1.0
           marker_array_vis.markers.append(marker_vis)

           marker_vis_number       = Marker() #marker
           marker_vis_number.header.frame_id = 'map'
           marker_vis_number.header.stamp = self.rospy_.Time.now()
           marker_vis_number.id           = i + 12
           marker_vis_number.type         = 9 #cylinder
           marker_vis_number.action       = 0 #add
           marker_vis_number.scale.x      = 0.2
           marker_vis_number.scale.y      = 0.2
           marker_vis_number.scale.z      = 1.0
           marker_vis_number.color.r = 1
           marker_vis_number.color.g = 1
           marker_vis_number.color.b = 1
           marker_vis_number.color.a = 1.0
           marker_vis_number.lifetime = self.rospy_.Duration(0.5)
           marker_vis_number.pose.position.x = self.pot_class[i].goal_pos_.pose.position.x
           marker_vis_number.pose.position.y = self.pot_class[i].goal_pos_.pose.position.y
           marker_vis_number.pose.position.z = 1.3
           marker_vis_number.pose.orientation.x = 0.0
           marker_vis_number.pose.orientation.y = 0.0
           marker_vis_number.pose.orientation.z = 0.0
           marker_vis_number.pose.orientation.w = 1.0
           marker_vis_number.text = "pot_"+str(i)
           marker_array_vis.markers.append(marker_vis_number)

       self.target_pub_.publish(marker_array_vis)

     def publish_detect_point(self): #ポータブルの目標座標を画面表示する
       marker_array_vis = MarkerArray() #people_visalizaion
       for i in range(1, 12):
         if self.pot_class[i].detect_register_key_ == True:
           marker_vis       = Marker() #marker
           marker_vis.header.frame_id = 'map'
           marker_vis.header.stamp = self.rospy_.Time.now()
           marker_vis.id           = i
           marker_vis.type         = 3 #cylinder
           marker_vis.action       = 0 #add
           marker_vis.scale.x      = 0.2
           marker_vis.scale.y      = 0.2
           marker_vis.scale.z      = 1.0
           marker_vis.color.r = 1
           marker_vis.color.g = 1
           marker_vis.color.b = 1
           marker_vis.color.a = 1.0
           marker_vis.lifetime = self.rospy_.Duration(0.5)
           marker_vis.pose.position.x = self.pot_class[i].detect_pos_.pose.position.x
           marker_vis.pose.position.y = self.pot_class[i].detect_pos_.pose.position.y
           marker_vis.pose.position.z = 0.3
           marker_vis.pose.orientation.x = 0.0
           marker_vis.pose.orientation.y = 0.0
           marker_vis.pose.orientation.z = 0.0
           marker_vis.pose.orientation.w = 1.0
           marker_array_vis.markers.append(marker_vis)

           marker_vis_number       = Marker() #marker
           marker_vis_number.header.frame_id = 'map'
           marker_vis_number.header.stamp = self.rospy_.Time.now()
           marker_vis_number.id           = i + 12
           marker_vis_number.type         = 9 #cylinder
           marker_vis_number.action       = 0 #add
           marker_vis_number.scale.x      = 0.2
           marker_vis_number.scale.y      = 0.2
           marker_vis_number.scale.z      = 1.0
           marker_vis_number.color.r = 1
           marker_vis_number.color.g = 1
           marker_vis_number.color.b = 1
           marker_vis_number.color.a = 1.0
           marker_vis_number.lifetime = self.rospy_.Duration(0.5)
           marker_vis_number.pose.position.x = self.pot_class[i].detect_pos_.pose.position.x
           marker_vis_number.pose.position.y = self.pot_class[i].detect_pos_.pose.position.y
           marker_vis_number.pose.position.z = 1.3
           marker_vis_number.pose.orientation.x = 0.0
           marker_vis_number.pose.orientation.y = 0.0
           marker_vis_number.pose.orientation.z = 0.0
           marker_vis_number.pose.orientation.w = 1.0
           marker_vis_number.text = "pot_"+str(i)
           marker_array_vis.markers.append(marker_vis_number)
       self.target_pub_.publish(marker_array_vis)

def main():
  rospy.init_node('portable_server')
  sd = PortableServerClass(rospy, tf)
  rate = rospy.Rate(10.0)
  host = '192.168.50.192'
  port = 4000
  backlog = 10
  bufsize = 4096*2*2*2

  server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  readfds = set([server_sock])

  #while not rospy.is_shutdown():
  #  sd.publish_detect_point() #ポータブルの目標位置を投げる
  #  sd.publish_target_point() #ポータブルの目標位置を投げる
  #  rate.sleep()

  try:
    server_sock.bind((host, port))
    server_sock.listen(backlog)

    while not rospy.is_shutdown():
      sd.publish_target_point() #ポータブルの目標位置を投げる
      sd.publish_detect_point() #ポータブルのdetect位置を投げる
      rready, wready, xready = select.select(readfds, [], [])
      for sock in rready:
        if sock is server_sock:
          conn, address = server_sock.accept()
          readfds.add(conn)
        else:
          msg = sock.recv(bufsize)
          if len(msg) == 0: #文字列の長さが0の時。
            sock.close()
            readfds.remove(sock)
          else:
            sd.string_analysis(msg) #送られてきた文字列を解析する
            sock.send(sd.msg_command) #目標値に移動等などコマンドを送る
            sd.publish_detect_point() #ポータブルの目標位置を投げる
            sd.publish_target_point() #ポータブルの目標位置を投げる
            if sd.rate_key_ == True:
              rate = rospy.Rate(10.0 * (sd.register_portable_))
              print("change" + str(10.0 * (sd.register_portable_)) + " hz")
              sd.rate_key_ = False
            sd.publish_portable_cloud_point() #車いすロボットの人避けに使う
            rate.sleep()
  finally:
    for sock in readfds:
      sock.close()
  return

if __name__ == '__main__':
  main()
