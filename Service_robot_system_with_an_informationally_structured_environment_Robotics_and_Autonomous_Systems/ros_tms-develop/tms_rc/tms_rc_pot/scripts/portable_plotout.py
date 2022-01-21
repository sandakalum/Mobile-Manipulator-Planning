#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import tf
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

def plot_out(args):
  #time.sleep(10)
  rospy.init_node('portable_plot_out', anonymous=True)
  start_position_pub   = rospy.Publisher('start_position', PoseWithCovarianceStamped, queue_size=10)
  goal_position_pub    = rospy.Publisher('goal_position', PoseStamped, queue_size=10)
  detect_position_pub  = rospy.Publisher('detect_position', PoseStamped, queue_size=10)

  if args[7] == 'start':
    rospy.loginfo(args[8] + "_start_position plotout")
    tmp_pose = PoseWithCovarianceStamped()
    num = 0
    r = rospy.Rate(5) # 10hz
    while num < 3:
      now         = rospy.Time.now()
      tmp_pose.header.stamp    = now
      tmp_pose.header.frame_id = args[8]
      tmp_pose.pose.pose.position.x = float(args[1]);
      tmp_pose.pose.pose.position.y = float(args[2]);
      tmp_pose.pose.pose.position.z = float(args[3]);
      quat = tf.transformations.quaternion_from_euler(float(args[6]), float(args[5]), float(args[4]))
      tmp_pose.pose.pose.orientation.x = quat[0];
      tmp_pose.pose.pose.orientation.y = quat[1];
      tmp_pose.pose.pose.orientation.z = quat[2];
      tmp_pose.pose.pose.orientation.w = quat[3];
      start_position_pub.publish(tmp_pose)
      num += 1
      r.sleep()

  elif args[7] == 'goal':
    rospy.loginfo(args[8] + "_goal_position plotout")
    tmp_pose = PoseStamped()
    num = 0
    r = rospy.Rate(5) # 10hz
    while num < 3:
      now      = rospy.Time.now()
      tmp_pose.header.stamp    = now
      tmp_pose.header.frame_id = args[8]
      tmp_pose.pose.position.x = float(args[1])
      tmp_pose.pose.position.y = float(args[2])
      tmp_pose.pose.position.z = float(args[3])
      quat = tf.transformations.quaternion_from_euler(float(args[6]), float(args[5]), float(args[4]))
      tmp_pose.pose.orientation.x = quat[0]
      tmp_pose.pose.orientation.y = quat[1]
      tmp_pose.pose.orientation.z = quat[2]
      tmp_pose.pose.orientation.w = quat[3]
      goal_position_pub.publish(tmp_pose)
      num += 1
      r.sleep()

  elif args[7] == 'detect':
    rospy.loginfo(args[8] + "_detect_position plotout")
    tmp_pose = PoseStamped()
    num = 0
    r = rospy.Rate(5) # 10hz
    while num < 3:
      now      = rospy.Time.now()
      tmp_pose.header.stamp    = now
      tmp_pose.header.frame_id = args[8]
      tmp_pose.pose.position.x = float(args[1])
      tmp_pose.pose.position.y = float(args[2])
      tmp_pose.pose.position.z = float(args[3])
      quat = tf.transformations.quaternion_from_euler(float(args[6]), float(args[5]), float(args[4]))
      tmp_pose.pose.orientation.x = quat[0]
      tmp_pose.pose.orientation.y = quat[1]
      tmp_pose.pose.orientation.z = quat[2]
      tmp_pose.pose.orientation.w = quat[3]
      detect_position_pub.publish(tmp_pose)
      num += 1
      r.sleep()

if __name__ == '__main__':
  try:
    plot_out(sys.argv)
  except rospy.ROSInterruptException: pass
