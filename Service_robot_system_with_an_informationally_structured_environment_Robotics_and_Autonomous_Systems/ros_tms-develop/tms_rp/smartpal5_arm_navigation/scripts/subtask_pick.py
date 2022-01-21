#!/usr/bin/env python

"""
    subtask_pick.py - Version 0.0.2 2015-09-20

    Command the gripper to grasp a target object and move it to a new location, all
    while avoiding simulated obstacles.

    Copyright 2014 by Patrick Goebel <patrick@pirobot.org, www.pirobot.org>
    Copyright 2015 by YS Pyo <passionvirus@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from tms_msg_db.msg import TmsdbStamped, Tmsdb
from tms_msg_db.srv import *
from tms_msg_rp.srv import *

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_FRAME = 'l_end_effector_link'
GRIPPER_OPEN = [-1.0]
GRIPPER_CLOSED = [-0.7]
GRIPPER_NEUTRAL = [-0.2]
GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

INIT_ARM_VALUE = [0.0, -0.17, 0.0, 0.0, 0.0, 0.0, 0.0]
GRASP_ARM_VALUE = [0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0]


class SubTaskPick:

    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('subtask_pick')
        rospy.on_shutdown(self.shutdown)

        self.pick_srv = rospy.Service('subtask_pick', rp_pick, self.pickSrvCallback)

    def pickSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)

        temp_dbdata = Tmsdb()
        target = Tmsdb()

        temp_dbdata.id = req.object_id
        temp_dbdata.state = 1

        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(temp_dbdata)
            target = res.tmsdb[0]
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            self.shutdown()

        print(target.name)

        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        # Give the scene a chance to catch up
        rospy.sleep(0.05)

        target_id = str(req.object_id)
        scene.remove_world_object(target_id)
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

        rospy.sleep(0.05)

        arm.set_named_target('l_arm_init')
        arm.go()
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()

        rospy.sleep(0.05)

        if target.offset_x < 0.01 and target.offset_y < 0.01 and target.offset_x < 0.01:
            target.offset_x = 0.033
            target.offset_y = 0.033
            target.offset_z = 0.083

        target_size = [(target.offset_x * 2), (target.offset_y * 2), (target.offset_z * 2)]
        # target_size = [0.03, 0.03, 0.12]
        # target_size = [0.07, 0.07, 0.14]
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = target.x
        target_pose.pose.position.y = target.y
        target_pose.pose.position.z = target.z + target.offset_z + 0.01
        # q = quaternion_from_euler(target.rr, target.rp, target.ry)
        q = quaternion_from_euler(0, 0, 0)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        scene.add_box(target_id, target_pose, target_size)

        rospy.sleep(0.05)

        print(target_pose.pose.position.x)
        print(target_pose.pose.position.y)
        print(target_pose.pose.position.z)

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose

        # Shift the grasp pose by half the width of the target to center it
        grasp_pose.pose.position.x -= target_size[0] / 2.0 + 0.01
        grasp_pose.pose.position.y -= target_size[1] / 2.0

        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])
        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.05)

        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " + str(n_attempts))
            result = arm.pick(target_id, grasps)
            print(result)
            rospy.sleep(0.02)
            if result != MoveItErrorCodes.SUCCESS:
                scene.remove_attached_object(GRIPPER_FRAME, target_id)

        ret = rp_pickResponse()
        # If the pick was successful, attempt the place operation
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Success the pick operation")
            ret.result = True
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
            ret.result = False

        return ret

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.1, 0.2, [0.0, 1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.2, [0.0, 0.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]

        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)

                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)

                # Append the grasp to the list
                grasps.append(deepcopy(g))

        # Return the list
        return grasps

    def shutdown(self):
        rospy.loginfo("Stopping the node")
        # Shut down MoveIt cleanly and exit the script
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        SubTaskPick()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("subtask_pick node terminated.")
