#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import cv2
from math import cos, sin, sqrt, pi
import numpy as np
import threading

from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import Imu
from tms_msg_rc.msg    import nucleo_serial

RADIUS    = 0.123  # タイヤの半径(人が乗った場合 ： 0.123, 手で押す場合 : 0.129)
TREAD     = 0.485  # タイヤ間距離
R_PULSE   = 4000   # エンコーダが一周した際のエンコーダ値（パルス数×4）

K_ENCODER = 2.0 * pi * RADIUS / R_PULSE
process_noise_std     = 5e-2 # process noise
measurement_noise_std = 1e-1 # measurement noise

class OdometryClass:
  def rad2deg(ang):
    return (ang*180.0/pi)

  def __init__(self):
    def sqr(var):
      return (var*var)

    self.odom_frame = rospy.get_param('~odom_frame_id', 'odom1')
    self.base_frame = rospy.get_param('~base_frame_id', 'base_footprint1')

    self.delta_r    = self.delta_l    = 0.0
    self.position_x = float(rospy.get_param('~initial_pose_x', 0.0))
    self.position_y = float(rospy.get_param('~initial_pose_y', 0.0))
    self.pose_yaw   = float(rospy.get_param('~initial_pose_a', 0.0))

    ######### kalman filter ############
    self.kalman = cv2.KalmanFilter(3, 2, 2)

    self.kalman.transitionMatrix = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    self.kalman.measurementMatrix = np.array([[1., 0., 0.], [0., 1., 0.]])
    self.kalman.processNoiseCov =sqr(process_noise_std) * np.eye(3)
    self.kalman.measurementNoiseCov = sqr(measurement_noise_std) * np.eye(2)
    self.kalman.errorCovPost = 1. * np.ones((3, 3))
    self.kalman.statePost = np.array([[self.position_x], [self.position_y], [self.pose_yaw]])
    self.kalman_updated = 0

    self.mutex = threading.Lock()
    ####################################

    self.LRF_sub     = rospy.Subscriber('/ninebot_measured_pos', Odometry, self.callback_LRF, queue_size=1)
    self.odom_pub    = rospy.Publisher('odom', Odometry, queue_size=1000)
    self.broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('nucleo_serial', nucleo_serial, self.callback_serial, queue_size=1)

  def callback_serial(self, data):
    if data.delta_t == 0: return

    self.delta_r = K_ENCODER * data.raw_delta_r
    self.delta_l = K_ENCODER * data.raw_delta_l

    ######### kalman filter ############
    self.mutex.acquire()
    control = np.array([[self.delta_r], [self.delta_l]])

    t = self.kalman.statePost[2, 0]
    self.kalman.controlMatrix = np.array([[cos(t) / 2.0, cos(t) / 2.0], [sin(t) / 2.0, sin(t) / 2.0], [1. / TREAD, -1. / TREAD]])

    prediction = self.kalman.predict(control)

    self.position_x = prediction[0, 0]
    self.position_y = prediction[1, 0]
    self.pose_yaw   = prediction[2, 0]
    while self.pose_yaw >  pi: self.pose_yaw -= 2.0 * pi
    while self.pose_yaw < -pi: self.pose_yaw += 2.0 * pi
    self.kalman_updated = 1
    self.mutex.release()
    ####################################

    #if kalman filter is not used, use simple integral or others
    # self.simple_integral(self.delta_r, self.delta_l)
    # self.precision_integral(self.delta_r, self.delta_l, data.delta_t, vel_theta)

    quate = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose_yaw)
    self.broadcaster.sendTransform( \
      (self.position_x, self.position_y, 0.0),  \
      (quate[0], quate[1], quate[2], quate[3]), \
      rospy.Time.now(), self.base_frame, self.odom_frame)
    odom                         = Odometry()
    odom.header.stamp            = rospy.Time.now()
    odom.header.frame_id         = self.odom_frame
    #odom.child_frame_id          = self.base_frame
    odom.pose.pose.position.x    = self.position_x
    odom.pose.pose.position.y    = self.position_y
    odom.pose.pose.orientation.x = quate[0]
    odom.pose.pose.orientation.y = quate[1]
    odom.pose.pose.orientation.z = quate[2]
    odom.pose.pose.orientation.w = quate[3]

    self.odom_pub.publish(odom)

  def callback_LRF(self, data):
    if self.kalman_updated == 0: return

    self.mutex.acquire()
    measurement = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y]])
    self.kalman.correct(measurement)
    self.mutex.release()

  def simple_integral(self, dr, dl):
    self.position_x += cos(self.pose_yaw) * (dr + dl) * 0.5
    self.position_y += sin(self.pose_yaw) * (dr + dl) * 0.5
    self.pose_yaw += (dr - dl) / TREAD
    while self.pose_yaw >  pi: self.pose_yaw -= 2.0 * pi
    while self.pose_yaw < -pi: self.pose_yaw += 2.0 * pi
  
  def precision_integral(self, dr, dl, dt, vel_theta):
    dth = vel_theta * dt
    if dth == 0:
      dx = ((dr + dl) * sin(dth * 0.5) * cos(self.pose_yaw + (dth * 0.5))) / dth
      dy = ((dr + dl) * sin(dth * 0.5) * sin(self.pose_yaw + (dth * 0.5))) / dth
    else:
      dx = (dr + dl) * cos(self.pose_yaw + (dth * 0.5)) * 0.5
      dy = (dr + dl) * sin(self.pose_yaw + (dth * 0.5)) * 0.5           
    self.position_x += dx
    self.position_y += dy

if __name__ == '__main__':
  rospy.init_node('ninebot_odometry', anonymous=True)
  try:
    try:
      RADIUS = float(rospy.get_param('~wheel_radius', RADIUS))
      K_ENCODER = 2.0 * pi * RADIUS / R_PULSE
    except: pass
    OdometryClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_odometry node finished.")
