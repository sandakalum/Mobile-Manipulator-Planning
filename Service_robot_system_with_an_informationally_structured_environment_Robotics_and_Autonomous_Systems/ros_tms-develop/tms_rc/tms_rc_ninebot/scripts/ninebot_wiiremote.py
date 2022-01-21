#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import time

from sensor_msgs.msg   import Joy
from geometry_msgs.msg import Twist

# wiiリモコンを接続するPC側で「rosrun wiimote wiimote_node.py」を実行すると、
# wiimote_nodeが出すtopicから、このノードがNinebotへの指令を生成する
# ninebot_smootherは通さず、直接角速度指令をninebot_serialへ送信している

ANGULAR_MAX    = 0.2  # 最終的に出力される角速度の絶対値の最大値
WIIREMOTE_PLAY = 0.08 # ヌンチャクのアナログパッドの遊び


class WiiRemoteClass:
  def __init__(self):
    rospy.init_node('ninebot_wiiremote', anonymous=True)
    rospy.on_shutdown(self.shutdown)
    topic_name = rospy.get_param('~cmd_vel_topic', 'serial_twist')
    
    self.twist_pub = rospy.Publisher(topic_name, Twist, queue_size=1)
    self.twist = Twist()
    self.twist.angular.z = 0
    self.twist_pub.publish(self.twist)

    rospy.Subscriber('/wiimote/nunchuk', Joy, self.callback, queue_size=1)
       
  def callback(self, data):

    value = self.limit(data.axes[0], -1.0, 1.0)
    
    if(abs(value) <  WIIREMOTE_PLAY): value = 0
    else:                             value = value * ANGULAR_MAX

    self.twist.angular.z = value
    self.twist_pub.publish(self.twist)
  
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
    WiiRemoteClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_wiiremote node finished.")