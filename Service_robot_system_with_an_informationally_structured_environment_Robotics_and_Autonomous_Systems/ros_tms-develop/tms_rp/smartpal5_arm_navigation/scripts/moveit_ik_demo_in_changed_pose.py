#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1.1 2015-08-26

    Use inverse kinemtatics to move the end effector to a specified pose

    Copyright 2014 by Patrick Goebel <patrick@pirobot.org, www.pirobot.org>
    Copyright 2015 by YS Pyo <passionvirus@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import sys
import moveit_commander
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_FRAME = 'l_end_effector_link'

GRIPPER_OPEN = [-1.0]
GRIPPER_CLOSED = [-0.3]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'


class MoveItFKDemo:

    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo', anonymous=True)

        robot = moveit_commander.RobotCommander()
        smartpal5_x_joint = moveit_commander.RobotCommander.Joint(robot, 'smartpal5_x_joint')
        smartpal5_y_joint = moveit_commander.RobotCommander.Joint(robot, 'smartpal5_y_joint')
        smartpal5_yaw_joint = moveit_commander.RobotCommander.Joint(robot, 'smartpal5_yaw_joint')

        # Use the groups of SmartPal5
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)
        end_effector_link = arm.get_end_effector_link()

        # Set a goal joint tolerance
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)

        # Set the option related IK solution
        arm.allow_replanning(True)

        # Use the pose stored in the SRDF file
        # 1. Set the target pose
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        arm.set_named_target('l_arm_init')
        arm.go()
        rospy.sleep(1)

        gripper.set_named_target('l_gripper_init')
        gripper.go()
        rospy.sleep(1)

        # Use the joint pose with IK
        # 1. Set the target pose for IK
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = smartpal5_x_joint.value() + 0.370748751767
        target_pose.pose.position.y = smartpal5_y_joint.value() + 0.420162677108
        target_pose.pose.position.z = 0.840162164512
        target_pose.pose.orientation.x = 0.0246617763278
        target_pose.pose.orientation.y = 0.024823036458
        target_pose.pose.orientation.z = -0.706731080872
        target_pose.pose.orientation.w = 0.706616439789

        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(1)

        # Shift the target pose (0,1,2,3,4,5 = x,y,z,r,p,y)
        # arm.shift_pose_target(1, -0.1, end_effector_link)
        # arm.go()
        # rospy.sleep(1)

        # gripper.set_named_target('l_gripper_open')
        # gripper.go()
        # rospy.sleep(1)

        gripper.set_named_target('l_gripper_init')
        gripper.go()
        rospy.sleep(1)

        # Return
        arm.set_named_target('l_arm_init')
        arm.go()
        rospy.sleep(1)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFKDemo()
    except rospy.ROSInterruptException:
        pass
