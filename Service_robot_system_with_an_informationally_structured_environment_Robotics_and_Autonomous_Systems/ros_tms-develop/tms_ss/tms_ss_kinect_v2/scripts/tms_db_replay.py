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


class TmsDbReplayer():

    def __init__(self):
        rospy.init_node("tms_db_replayer")
        rospy.on_shutdown(self.shutdown)

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        if not self.is_connected:
            raise Exception("Problem of connection")

        self.data_pub = rospy.Publisher('tms_db_replayer', TmsdbStamped, queue_size=10)

        self.sendDbHistoryInformation()

    def sendDbHistoryInformation(self):
        rate = rospy.Rate(100)  # 100hz

        while not rospy.is_shutdown():
            temp_dbdata = Tmsdb()
            object_information = TmsdbStamped()

            d = datetime.now() - timedelta(days=3, hours=11)
            t1 = 'ISODate(' + d.isoformat() + ')'
            t2 = 'ISODate(' + (d + timedelta(days=0, hours=1, minutes=0)).isoformat() + ')'
            print(t1)
            # print(db.history.ensure_index({'id':1,'time':1}))
            # cursor = db.history.find({'id':{'$gte':1006, '$lte':1018},
            # 'time':{'$gte':t1,'$lt':t2}, 'state':1}).sort('time')
            cursor = db.history.find({'id': {'$gte': 1006, '$lte': 1018}, 'time': {'$gte': t1, '$lt': t2}, 'state': 1})
            for doc in cursor:
                # if not rospy.is_shutdown():
                #    print(doc)
                del doc['_id']
                temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                object_information.tmsdb.append(temp_dbdata)
            self.data_pub.publish(object_information)

            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbReplayer()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_replayer node terminated.")
