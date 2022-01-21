#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('plot_out', anonymous=True)
    tmp_pose = PoseStamped()

    time.sleep(5.0)
    num = 0
    r = rospy.Rate(5) # 10hz
    while num < 50:
      now      = rospy.Time.now()
      tmp_pose.header.stamp    = now
      tmp_pose.header.frame_id = "map11"
      tmp_pose.pose.position.x = 8.266  #11.3206233978
      tmp_pose.pose.position.y = -10.948  #-11.5605945587
      tmp_pose.pose.position.z = 0.0
      tmp_pose.pose.orientation.x = 0.0
      tmp_pose.pose.orientation.y = 0.0
      tmp_pose.pose.orientation.z = 0.94107098415
      tmp_pose.pose.orientation.w = -0.33820911104
      pub.publish(tmp_pose)
      num += 1
      print num,  ",\r"
      r.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
