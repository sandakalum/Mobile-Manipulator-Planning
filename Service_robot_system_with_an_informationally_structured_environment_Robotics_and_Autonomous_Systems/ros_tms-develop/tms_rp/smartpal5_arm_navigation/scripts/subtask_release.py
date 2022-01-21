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
GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

GRIPPER_OPEN_VAL = [-1.0]


class SubTaskRelease:

    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('subtask_release')
        rospy.on_shutdown(self.shutdown)

        self.release_srv = rospy.Service('subtask_release', rp_release, self.ReleaseSrvCallback)

    def ReleaseSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)

        arm = MoveGroupCommander(GROUP_NAME_ARM)
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        scene = PlanningSceneInterface()

        rospy.sleep(0.1)

        result = None

        gripper.set_joint_value_target(GRIPPER_OPEN_VAL)
        result = gripper.go()
        arm.detach_object()
        gripper.detach_object()
        scene.remove_attached_object(GRIPPER_FRAME, str(req.object_id))
        scene.remove_world_object(str(req.object_id))
        result = True

        ret = rp_releaseResponse()
        ret.result = result
        return ret

    def shutdown(self):
        rospy.loginfo("Stopping the node")
        # Shut down MoveIt cleanly and exit the script
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        SubTaskRelease()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("subtask_release node terminated.")
