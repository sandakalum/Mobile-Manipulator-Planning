#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import genpy
import pymongo  # https://api.mongodb.org/python/2.6.3/
import json
import copy
from bson import json_util
from bson.objectid import ObjectId
from datetime import *
from tms_msg_db.msg import TmsdbStamped, Tmsdb
import tms_db_manager.tms_db_util as db_util

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb


class TmsDbBackuper():

    def __init__(self):
        rospy.init_node("tms_db_backuper")
        rospy.on_shutdown(self.shutdown)
        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        if not self.is_connected:
            raise Exception("Problem of connection")

        rospy.Timer(rospy.Duration(60), self.manageDataCallback)  # 60sec
        rospy.Timer(rospy.Duration(12 * 60 * 60), self.removeForeverDataCallback)  # 12hour

    def manageDataCallback(self, event):
        # GMT +9 -1 hour
        nowtime = rospy.Time.now() + rospy.Duration(9 * 60 * 60) - rospy.Duration(1 * 60 * 60)
        update_period = str(datetime.utcfromtimestamp(nowtime.to_sec()))
        # print(update_period)

        cursor = db.history.find({'time': {'$lt': update_period}})
        # print(cursor.count())
        for doc in cursor:
            db.backup.insert(doc)

        db.history.remove({'time': {'$lt': update_period}})

    def removeForeverDataCallback(self, event):
        # GMT +9hour - 14day
        nowtime = rospy.Time.now() + rospy.Duration(9 * 60 * 60) - rospy.Duration(14 * 24 * 60 * 60)
        update_period = str(datetime.utcfromtimestamp(nowtime.to_sec()))
        # print(update_period)
        db.backup.remove({'time': {'$lt': update_period}})

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbBackuper()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_backuper node terminated.")

# test mongo Shell
# db.getCollection('history').find({'time':{ $lt: "2015-09-01 15:05:34.596954"}}).count()
# db.getCollection('history').find().sort({'time':-1}) e.g. 1 for ascending and -1 for descending.
