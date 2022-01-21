#!/usr/bin/env python

import roslib
import rospy
import tf

import math
import sys, os

#from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import People
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# In: People: people_ninebot_cropped / frame_id map1 
#     base_frame
# Out: PoseStamped: move_base_simple/goal frame_id map1

# constants
DIST_MIN = .3 # how close is too close that robot won't send a new goal
DIST_MAX = 6 # how far is too far that robot should not consider it as new person(default:3)
ANGLE_THRESHOLD = math.pi / 6 # how wide is too wide robot will send new goal
DIST_FROM_TARGET = .5 # how far away the robot should stop from the target
PROXIMITY_MAX = .4 # how far from last known position leg detector should consider to be probable
R_SCALE = .5 # scale from distance to reliability
RELIABILITY_MIN = .4 #minimum reliability of the position

class ListenerSingleton:
    created = False
    listener = None

    @staticmethod
    def new():
        if (ListenerSingleton.created):
            return ListenerSingleton.listener
        else:
            ListenerSingleton.created = True
            ListenerSingleton.listener = tf.TransformListener()
            rospy.loginfo("created new instance of listener")
            return ListenerSingleton.listener    

class GoalEuler:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class HumanFollower:

    def __init__(self):
        self.global_frame = rospy.get_param('~global_frame_id', 'map1')
        self.base_frame   = rospy.get_param('~base_frame_id',   'base_footprint1')
        pose_topic_name   = rospy.get_param('~pose_topic_name',   '/ninebot_pos')
        self.people_topic_name = rospy.get_param('~people_topic_name', '/people_ninebot_cropped')
        goal_topic_name   = rospy.get_param('~goal_topic_name', '/move_base_simple/goal')

        self.pub = rospy.Publisher(goal_topic_name, PoseStamped, queue_size = 10)
        # this can be deleted
        self.positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)

        self.previousGoal = None
        self.lastKnownPosition = None


    def callback(self, data):
        # get transform
        listener = ListenerSingleton.new()

        try:
          #(trans, rot) = listener.lookupTransform('/map1', '/base_footprint1', rospy.Time())
          (trans, rot) = listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): return

        # rospy.loginfo("Transform obtained")

        # sends current position for visualization
        self.sendCurrentPosition(trans, rot)

        # process leg detector input
        if len(data.people) > 0:
            personIndex = self.findReliableTarget(data, trans)

            # found someone more probable than the min probability.
            if (personIndex != -1):
                #rospy.loginfo("Target Found")
                
                try:

                    # logs the start of goal computation
                    #rospy.loginfo("Computing goal")

                    # This is where the target person's position are                                      
                    PersonPositionX = data.people[personIndex].position.x
                    PersonPositionY = data.people[personIndex].position.y

                    # setting last known position regardless of if the goal is sent or not
                    # angle is not important. Last Known position only needs the coordinates
                    self.lastKnownPosition = GoalEuler(PersonPositionX, PersonPositionY, 0)                    

                    # computing target point that is set distance away    
                    differenceX = PersonPositionX - trans[0]
                    differenceY = PersonPositionY - trans[1]
                    
                    # calculating target location
                    goalAngle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    # calculating the position of the goal
                    target_length = length - DIST_FROM_TARGET
                    goalX = target_length * math.cos(goalAngle) + trans[0]
                    goalY = target_length * math.sin(goalAngle) + trans[1]

                    #rospy.loginfo("Person: " + str(PersonPositionX) + " " + str(PersonPositionY))
                    #rospy.loginfo("trans: " + str(trans[0]) + " " + str(trans[1]))
                    #rospy.loginfo("Goal: " + str(goalX) + " " + str(goalY))

                    # sending goal if it is sufficiently different or the first goal
                    #rospy.loginfo("judging goal")
                    if (self.previousGoal == None or self.checkGoalDifference(goalX, goalY, goalAngle)):

                        self.previousGoal = GoalEuler(goalX, goalY, goalAngle)

                        target_goal_simple = self.buildGoalQuaternion(goalX, goalY, goalAngle) 

                        #rospy.loginfo("*********************************")
                        #rospy.loginfo("sending goal")
                        self.pub.publish(target_goal_simple)
                    #else:
                        #rospy.loginfo("new goal not sufficiently different. Canclled.")
                
                except Exception as expt:
                    #exc_type, exc_obj, exc_tb = sys.exc_info()
                    #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    #print(exc_type, fname, exc_tb.tb_lineno)
                    #print type(expt)
                    print expt.args
                

    def buildGoalQuaternion(self, goalX, goalY, goalAngle):
        #rospy.loginfo("building final goal")
        # calculating the quaterion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, goalAngle)

        # forming target goal
        goal = PoseStamped()
        
        goal.pose.position.x = goalX
        goal.pose.position.y = goalY
        goal.pose.position.z = 0
        
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        goal.header.frame_id = self.global_frame
        goal.header.stamp = rospy.Time.now()

        return goal

    def checkGoalDifference(self, goalX, goalY, goalAngle):
        # check if distance is far enough
        distDiff = math.hypot(goalX - self.previousGoal.x, goalY - self.previousGoal.y)
        angleDiff = math.fabs(goalAngle - self.previousGoal.angle)
        
        # if either is greather than threshold, we should send new goal
        return (distDiff > DIST_MIN or angleDiff > ANGLE_THRESHOLD)
            
    def findReliableTarget(self, data, roboPosition):
        # selecting most probable person
        #rospy.loginfo("Filtering for suitible target")

        maxReliability = RELIABILITY_MIN
        reliability = 0
        personIndex = -1

        for i in range(len(data.people)):
            
            # reliability metric is based on a combination of leg_detector results
            # and how far this current goal is from the pervious goal.
            # if the same person is still in sight, it is the most reliable
            # If there is no previous goal, then it's simply the leg_detector reliability

            currPersonPositionX = data.people[i].position.x
            currPersonPositionY = data.people[i].position.y

            if(data.people[i].reliability == 0 ):
                distFromRobot = math.hypot(currPersonPositionX - roboPosition[0], currPersonPositionY - roboPosition[1])
                # to ignore dist=0
                data.people[i].reliability = 1.0 / (distFromRobot + 1.0) + RELIABILITY_MIN 
 
            if (self.previousGoal == None):
                reliability = data.people[i].reliability
            else: 
                distFromRobot = math.hypot(currPersonPositionX - roboPosition[0], currPersonPositionY - roboPosition[1])
                distFromLastX = currPersonPositionX - self.lastKnownPosition.x
                distFromLastY = currPersonPositionY - self.lastKnownPosition.y
                distFromLastKnown = math.hypot(distFromLastX, distFromLastY)

                if (distFromRobot > DIST_MAX):
                    reliability = -100
                else:
                    # general case not the first goal
                    if (distFromLastKnown < PROXIMITY_MAX):
                        reliability = data.people[i].reliability + ((PROXIMITY_MAX - distFromLastKnown) * R_SCALE)
                    else:
                        reliability = data.people[i].reliability


            if (reliability > maxReliability):
                maxReliability = reliability
                personIndex = i

        #rospy.loginfo("count: " + str(len(data.people)))
        #rospy.loginfo("final R: " + str(reliability))
        #rospy.loginfo("DIST: " + str(distFromRobot))
        #rospy.loginfo("currPersonPosition: " + str(currPersonPositionX) + " " + str(currPersonPositionY))
        #rospy.loginfo("roboPosition: " + str(roboPosition[0]) + " " + str(roboPosition[1]))

        return personIndex


    def sendCurrentPosition(self, trans, rot):
        curr_Position = PoseStamped()
        curr_Position.pose.position.x = trans[0]
        curr_Position.pose.position.y = trans[0]
        curr_Position.pose.position.z = 0
        curr_Position.pose.orientation.x = rot[0]
        curr_Position.pose.orientation.y = rot[1]
        curr_Position.pose.orientation.z = rot[2]
        curr_Position.pose.orientation.w = rot[3]
        curr_Position.header.frame_id = self.global_frame
        curr_Position.header.stamp = rospy.Time.now()

        # publishing current position for visualization
        self.positionPub.publish(curr_Position)

    def run(self):
        rospy.init_node("human_follower")
        rospy.Subscriber(self.people_topic_name, People, self.callback)
        rospy.spin()        

if __name__ == '__main__':
    try:
        hf = HumanFollower()
        hf.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("oh no, he's dead!")

