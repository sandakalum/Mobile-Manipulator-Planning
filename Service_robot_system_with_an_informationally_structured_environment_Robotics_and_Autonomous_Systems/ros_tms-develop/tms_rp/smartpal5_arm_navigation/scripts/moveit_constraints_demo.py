#!/usr/bin/env python

"""
    moveit_constraints_demo.py - Version 0.1.1 2015-08-26

    Move the gripper while maintaining a given orientation

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
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_OPEN = [-1.0]
GRIPPER_CLOSED = [-0.3]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'


class MoveItDemo:

    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_constraints_demo', anonymous=True)

        robot = RobotCommander()

        # Connect to the arm move group
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Increase the planning time since constraint planning can take a while
        arm.set_planning_time(5)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow some leeway in position(meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Start in the "resting" configuration stored in the SRDF file
        arm.set_named_target('l_arm_init')

        # Plan and execute a trajectory to the goal configuration
        arm.go()
        rospy.sleep(1)

        # Open the gripper
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()
        rospy.sleep(1)

        # Set an initial target pose with the arm up and to the right
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.263803774718
        target_pose.pose.position.y = 0.295405791959
        target_pose.pose.position.z = 0.690438884208
        q = quaternion_from_euler(0, 0, -1.57079633)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Set the start state and target pose, then plan and execute
        arm.set_start_state(robot.get_current_state())
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(2)

        # Close the gripper
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(1)

        # Store the current pose
        start_pose = arm.get_current_pose(end_effector_link)

        # Create a contraints list and give it a name
        constraints = Constraints()
        constraints.name = "Keep gripper horizontal"

        # Create an orientation constraint for the right gripper
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = start_pose.header
        orientation_constraint.link_name = arm.get_end_effector_link()
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        # q = quaternion_from_euler(0, 0, -1.57079633)
        # orientation_constraint.orientation.x = q[0]
        # orientation_constraint.orientation.y = q[1]
        # orientation_constraint.orientation.z = q[2]
        # orientation_constraint.orientation.w = q[3]

        # Append the constraint to the list of contraints
        constraints.orientation_constraints.append(orientation_constraint)

        # Set the path constraints on the arm
        arm.set_path_constraints(constraints)

        # Set a target pose for the arm
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.39000848183
        target_pose.pose.position.y = 0.185900663329
        target_pose.pose.position.z = 0.732752341378
        target_pose.pose.orientation.w = 1

        # Set the start state and target pose, then plan and execute
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(1)

        # Clear all path constraints
        arm.clear_path_constraints()

        # Open the gripper
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()
        rospy.sleep(1)

        # Return to the "resting" configuration stored in the SRDF file
        arm.set_named_target('l_arm_init')

        # Plan and execute a trajectory to the goal configuration
        arm.go()
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
