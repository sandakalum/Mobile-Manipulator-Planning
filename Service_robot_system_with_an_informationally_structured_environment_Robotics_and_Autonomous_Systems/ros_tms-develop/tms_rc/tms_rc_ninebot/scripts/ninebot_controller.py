#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys
import time
import rospy
import tf
import wiringpi2

from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg  import Twist
from std_srvs.srv       import Empty
from tms_msg_rc.srv     import nav_command
from tms_msg_rc.srv     import nav_commandResponse

class LedControlClass:
  def __init__(self):
    self.pins = [2, 7, 3] #GPIO
    self.ryb  = [0, 0, 0] #Red Yellow Blue
    wiringpi2.wiringPiSetup()

    for x in self.pins:
      wiringpi2.pinMode(x, 1)
      wiringpi2.digitalWrite(x, 1)
    time.sleep(2.5)
    for x in self.pins:
      wiringpi2.digitalWrite(x, 0)
    time.sleep(0.5)
    self.ryb = [0, 0, 1]

  def update(self):
    wiringpi2.digitalWrite(self.pins[0], self.ryb[0])
    wiringpi2.digitalWrite(self.pins[1], self.ryb[1])
    wiringpi2.digitalWrite(self.pins[2], self.ryb[2])

class ControllerClass:
  def __init__(self, args):
    rospy.init_node('ninebot_controller')
    rospy.on_shutdown(self.shutdown)

    self.goal         = PoseStamped()
    self.leds         = LedControlClass()
    self.global_frame = rospy.get_param('~global_frame_id', 'map1')

    if not rospy.get_param('~move_base', True): self.Led_controll()

    self.start_pub  = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    self.goal_pub   = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    self.cmd_pub    = rospy.Publisher('serial_twist', Twist, queue_size=10)

    rospy.Service('ninebot_command', nav_command, self.callback_command)
    rospy.wait_for_service('move_base/clear_costmaps')
    self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.callback_start, queue_size=10)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback_goal, queue_size=10)
    rospy.Subscriber('move_base/status', GoalStatusArray, self.callback_status, queue_size=1)
    self.Led_controll()

  def Led_controll(self):
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
      self.leds.update()
      rate.sleep()

  def callback_command(self, data):
    rospy.loginfo("command_" + str(data.command))
    if data.command == 'start':
      self.cancel_plotout()
      quate = tf.transformations.quaternion_from_euler(0, 0, data.a)
      start                         = PoseWithCovarianceStamped()
      start.header.stamp            = rospy.Time.now()
      start.header.frame_id         = self.global_frame
      start.pose.pose.position.x    = data.x
      start.pose.pose.position.y    = data.y
      start.pose.pose.position.z    = 0.0
      start.pose.pose.orientation.x = quate[0]
      start.pose.pose.orientation.y = quate[1]
      start.pose.pose.orientation.z = quate[2]
      start.pose.pose.orientation.w = quate[3]
      self.start_plotout(start)
    elif data.command == 'goal':
      quate = tf.transformations.quaternion_from_euler(0, 0, data.a)
      self.goal                    = PoseStamped()
      self.goal.header.stamp       = rospy.Time.now()
      self.goal.header.frame_id    = self.global_frame
      self.goal.pose.position.x    = data.x
      self.goal.pose.position.y    = data.y
      self.goal.pose.position.z    = 0.0
      self.goal.pose.orientation.x = quate[0]
      self.goal.pose.orientation.y = quate[1]
      self.goal.pose.orientation.z = quate[2]
      self.goal.pose.orientation.w = quate[3]
      self.goal_plotout(self.goal)
    elif data.command == 'cancel':
      self.cancel_plotout()
    else:
      return nav_commandResponse('error : nav_command')
    return nav_commandResponse('success : nav_command')

  def callback_status(self, data):
    
    if len(data.status_list) <= 0: return
    status = int(data.status_list[-1].status)
    
    if   status == 0: self.leds.ryb = [0, 0, 1] #pending
    elif status == 1: self.leds.ryb = [0, 1, 0] #active
    elif status == 2: self.leds.ryb = [0, 0, 1] #preempted
    elif status == 3: self.leds.ryb = [1, 1, 1] #succeeded
    elif status == 4:                           #aborted
      self.leds.ryb = [1, 0, 0]     
      time.sleep(1.0)
      try:
        self.sound_command()
        self.clear_costmaps()
        self.goal_plotout(self.goal)
      except rospy.ServiceException, e:
        rospy.logerr("Clearing costmap service call failed: %s"%e)
    else: self.leds.ryb = [1, 0, 1]

  def callback_start(self, data):
    self.clear_costmaps()

  def callback_goal(self, data):
    self.goal = data
  
  def start_plotout(self, data):
    for i in range(0, 5):
      self.start_pub.publish(data)

  def goal_plotout(self, data):
    for i in range(0, 5):
      self.goal_pub.publish(data)

  def cancel_plotout(self):
    self.clear_costmaps()
    for i in range(0, 5):
      self.cancel_pub.publish(GoalID())

  def sound_command(self):
    sound = Twist()
    sound.linear.z = 1.0
    for i in range(0, 15):
      self.cmd_pub.publish(sound)

  def shutdown(self):
    try:
      for x in self.leds.pins:
        wiringpi2.digitalWrite(x, 0)
      self.cancel_pub.publish(GoalID())
    except: pass

if __name__ == '__main__':
  try:
    ControllerClass(sys.argv)
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_controller node finished.")
