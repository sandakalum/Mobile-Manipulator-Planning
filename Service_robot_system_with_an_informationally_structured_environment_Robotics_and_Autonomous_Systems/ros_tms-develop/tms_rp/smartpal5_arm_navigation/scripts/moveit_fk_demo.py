#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1.1 2015-08-26

    Use forward kinemtatics to move the arm to a specified set of joint angles

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

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'


class MoveItFKDemo:

    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # Use the groups of SmartPal5
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper_left = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Set a goal joint tolerance
        arm.set_goal_joint_tolerance(0.001)
        gripper_left.set_goal_joint_tolerance(0.001)

        # Use the pose stored in the SRDF file
        # 1. Set the target pose
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        arm.set_named_target('l_arm_init')
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(1)

        gripper_left.set_named_target('l_gripper_init')
        gripper_left.go()
        rospy.sleep(1)

        # Use the joint positions with FK
        # 1. Set the target joint values
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        joint_positions = [0.0, 0.07, 0.0, 1.5707, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)

        gripper_left.set_named_target('l_gripper_open')
        gripper_left.go()
        rospy.sleep(1)

        # Return
        arm.set_named_target('l_arm_init')
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(1)

        gripper_left.set_named_target('l_gripper_init')
        gripper_left.go()
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
