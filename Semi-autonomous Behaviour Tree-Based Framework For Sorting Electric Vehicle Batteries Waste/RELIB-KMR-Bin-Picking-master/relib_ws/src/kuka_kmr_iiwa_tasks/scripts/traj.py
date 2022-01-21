#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
	rospy.init_node("traj", anonymous=True)

	pub = rospy.Publisher("/lbr_controller/arm_controller/command", JointTrajectory, queue_size=10)

	rate = rospy.Rate(0.2)

	msg = JointTrajectory()
	msg.joint_names = ("joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7")

	jtp = JointTrajectoryPoint()
	jtp.time_from_start.secs = 0.01
	jtp.positions = [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0]
	msg.points.append(jtp)



	while not rospy.is_shutdown():
		jtp.positions[2] = 0.2
		for i in range(3):
			jtp.positions[2] = jtp.positions[2] + 0.15
			msg.points.append(jtp)
		
		print("NEW")
		pub.publish(msg)
		rate.sleep()