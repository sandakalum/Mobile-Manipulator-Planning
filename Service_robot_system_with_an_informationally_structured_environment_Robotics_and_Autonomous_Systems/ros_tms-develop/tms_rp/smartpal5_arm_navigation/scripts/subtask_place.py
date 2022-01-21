#!/usr/bin/env python

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
GRIPPER_NEUTRAL = [0.0]
GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

INIT_ARM_VALUE = [0.0, -0.08, 0.0, 0.0, 0.0, 0.0, 0.0]


class SubTaskPlace:

    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('subtask_place')
        rospy.on_shutdown(self.shutdown)

        self.place_srv = rospy.Service('subtask_place', rp_place, self.placeSrvCallback)

    def placeSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)

        temp_dbdata = Tmsdb()
        target = Tmsdb()

        temp_dbdata.id = req.object_id

        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(temp_dbdata)
            target = res.tmsdb[0]
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            self.shutdown()

        print(target.name)

        scene = PlanningSceneInterface()

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
        arm.set_goal_position_tolerance(0.1)
        arm.set_goal_orientation_tolerance(0.3)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)
        # Set a limit on the number of place attempts
        max_place_attempts = 5
        # Give the scene a chance to catch up
        rospy.sleep(0.05)

        target_id = str(req.object_id)

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = req.x
        target_pose.pose.position.y = req.y
        target_pose.pose.position.z = req.z
        # q = quaternion_from_euler(target.rr, target.rp, target.ry)
        q = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        print(target_pose.pose.position.x)
        print(target_pose.pose.position.y)
        print(target_pose.pose.position.z)
        print(target_pose.pose.orientation.x)
        print(target_pose.pose.orientation.y)
        print(target_pose.pose.orientation.z)
        print(target_pose.pose.orientation.w)

        # Initialize the grasp pose to the target pose
        place_pose = target_pose

        # Generate a list of grasps
        places = self.make_places(place_pose)

        result = None
        n_attempts = 0
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
            n_attempts += 1
            rospy.loginfo("Place attempt: " + str(n_attempts))
            for place in places:
                result = arm.place(target_id, place)
                print(result)
                if result == MoveItErrorCodes.SUCCESS:
                    break
            # rospy.sleep(0.2)

        scene.remove_world_object(str(req.object_id))

        ret = rp_placeResponse()
        # If the pick was successful, attempt the place operation
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Success the place operation")
            ret.result = True
        else:
            rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
            ret.result = False

        return ret

    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.01, 0.02, -0.01, -0.02]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.01, 0.02, -0.01, -0.02]

        roll_vals = [0]

        pitch_vals = [0]

        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for y in y_vals:
            for x in x_vals:
                place.pose.position.x = init_pose.pose.position.x + x
                place.pose.position.y = init_pose.pose.position.y + y

                # q = quaternion_from_euler(0, p, yaw)
                # place.pose.orientation.x = init_pose.pose.orientation.x
                # place.pose.orientation.y = init_pose.pose.orientation.y
                # place.pose.orientation.z = init_pose.pose.orientation.z
                # place.pose.orientation.w = init_pose.pose.orientation.w

                # Append this place pose to the list
                places.append(deepcopy(place))

        # Return the list
        return places

    def shutdown(self):
        rospy.loginfo("Stopping the node")
        # Shut down MoveIt cleanly and exit the script
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        SubTaskPlace()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("subtask_place node terminated.")
