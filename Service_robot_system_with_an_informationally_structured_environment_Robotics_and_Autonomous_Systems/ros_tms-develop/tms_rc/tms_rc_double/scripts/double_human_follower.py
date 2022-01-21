#!/usr/bin/python
# -*- coding: utf-8 -*-
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import ts_req
import rospy
import requests
import time
import subprocess
import shlex
import json
import datetime
import threading
import urllib
import numpy as np

task_id = 8008
robot_id = 2012
object_id = 0
user_id = 1100
place_id = 0

class double_human_follower():

    def __init__(self):
        rospy.init_node("double_human_follower")
        rospy.on_shutdown(self.shutdown)
	
	self.user_place = self.getPose()
	self.timer = threading.Timer(20,self.follow)
	self.timer.start()
	
        

    def follow(self):
	if self.cal_distance(self.user_place,self.getPose()):
		print 'send command'
        	try:
            		rospy.wait_for_service('tms_ts_master', timeout=1.0)
      	 	except rospy.ROSException as e:
   	        	print "tms_ts_master is not work"
			self.timer = threading.Timer(20,self.follow)
			self.timer.start()
			return

      	  	try:
            		tms_ts_master = rospy.ServiceProxy('tms_ts_master',ts_req)
            		res = tms_ts_master(0,task_id,robot_id,object_id,user_id,place_id,0)
            		print res
        	except rospy.ServiceException as e:
            		print "Service call failed: %s" % e
		self.user_place = self.getPose()
	self.timer = threading.Timer(10,self.follow)
	self.timer.start()

    def db_reader(self,data):
        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(data)
            return res
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            return None

    def getPose(self):
        user_dbdata = Tmsdb()
        user_dbdata.id  = 1100
        user_dbdata.state = 1
        user_data = self.db_reader(user_dbdata)
        if user_data is None:
            print "failed to get user data"

#        robot_dbdata = Tmsdb()
#        robot_dbdata.id  = 2012
#        robot_dbdata.sensor = 3001
#        robot_dbdata.state = 1
#        robot_data = self.db_reader(robot_dbdata)
#        if robot_data is None:
#            print "failed to get robot data"

#	print user_data.tmsdb[0].x

#	distance = np.sqrt(np.square(self.user_place_x - user_data.tmsdb[0].x) + np.square(self.user_place_y - user_data.tmsdb[0].y))
#	user_place_x = user_data.tmsdb[0].x
#	user_place_y = user_data.tmsdb[0].y

#	if distance > 2:
#		return True
#	else:
#		return False
	return user_data	


    
    def cal_distance(self,data1,data2):
	
	distance = np.sqrt(np.square(data1.tmsdb[0].x - data2.tmsdb[0].x) + np.square(data1.tmsdb[0].y - data2.tmsdb[0].y))
	
	if distance > 1:
		return True
	else :
		return False
    def shutdown(self):
        rospy.loginfo("Stopping the node")


if __name__ == '__main__':
    follower = double_human_follower()
    #follower.follow()
	#rospy.spin()
#    try:
#        follow()
#        rospy.spin()
#    except rospy.ROSInterruptException:
#        rospy.loginfo("double_human_follower node terminated.")
