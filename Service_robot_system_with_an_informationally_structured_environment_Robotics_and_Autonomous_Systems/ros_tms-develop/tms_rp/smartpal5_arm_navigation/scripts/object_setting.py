#!/usr/bin/env python
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from tf.transformations import quaternion_from_euler
from tms_msg_db.msg import TmsdbStamped, Tmsdb
from tms_msg_db.srv import *

# http://docs.ros.org/indigo/api/moveit_python/html/

REFERENCE_FRAME = 'world_link'


class ObjectSetting:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('object_setting')
        rospy.on_shutdown(self.shutdown)

        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.colors = dict()

        scene = PlanningSceneInterface()

        rospy.sleep(1)

        temp_dbdata = Tmsdb()
        temp_dbdata.name = 'chipstar_red'

        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(temp_dbdata)
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            self.shutdown()

        print(res.tmsdb)

        target_id = 'chipstar_red'
        scene.remove_world_object(target_id)
        scene.remove_attached_object("l_end_effector_link", target_id)

        target_size = [(res.tmsdb[0].offset_x * 2), (res.tmsdb[0].offset_y * 2), (res.tmsdb[0].offset_z * 2)]
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = res.tmsdb[0].x
        target_pose.pose.position.y = res.tmsdb[0].y
        target_pose.pose.position.z = res.tmsdb[0].z + res.tmsdb[0].offset_z
        q = quaternion_from_euler(res.tmsdb[0].rr, res.tmsdb[0].rp, res.tmsdb[0].ry)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        scene.add_box(target_id, target_pose, target_size)

        self.setColor(target_id, 1, 0, 0, 1.0)
        self.sendColors()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == "__main__":
    try:
        ObjectSetting()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_setting node terminated.")
