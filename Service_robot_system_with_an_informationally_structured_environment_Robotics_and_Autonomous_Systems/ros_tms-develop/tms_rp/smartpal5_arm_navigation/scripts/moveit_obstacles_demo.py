#!/usr/bin/env python

"""
    moveit_obstacles_demo.py - Version 0.1.1 2015-08-26

    Move the gripper to a target pose while avoiding simulated obstacles.

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
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_FRAME = 'l_end_effector_link'

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

        rospy.init_node('moveit_obstacles_demo')

        # Construct the initial scene object
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Pause for the scene to get ready
        rospy.sleep(1)

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame accordingly
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'

        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)

        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Start the arm in the "resting" pose stored in the SRDF file
        arm.set_named_target('l_arm_init')
        arm.go()

        rospy.sleep(2)

        # Set the height of the table off the ground
        table_ground = 0.65

        # Set the length, width and height of the table and boxes
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]

        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.35
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = 0.3
        box1_pose.pose.position.y = 0
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0
        scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = 0.3
        box2_pose.pose.position.y = 0.25
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0
        scene.add_box(box2_id, box2_pose, box2_size)

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)

        # Send the colors to the planning scene
        self.sendColors()

        # Set the target pose in between the boxes and above the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.22
        target_pose.pose.position.y = 0.14
        target_pose.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.05

        q = quaternion_from_euler(0, 0, -1.57079633)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Set the target pose for the arm
        arm.set_pose_target(target_pose, end_effector_link)

        # Move the arm to the target pose (if possible)
        arm.go()

        # Pause for a moment...
        rospy.sleep(2)

        # Return the arm to the "resting" pose stored in the SRDF file
        arm.set_named_target('l_arm_init')
        arm.go()

        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

    # Set the color of an object
    def setColor(self, name, r, g, b, a=0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
