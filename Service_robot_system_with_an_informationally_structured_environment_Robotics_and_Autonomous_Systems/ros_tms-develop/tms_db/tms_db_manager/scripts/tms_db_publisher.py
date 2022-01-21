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


class TmsDbPublisher():

    def __init__(self):
        rospy.init_node("tms_db_publisher")
        rospy.on_shutdown(self.shutdown)

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        if not self.is_connected:
            raise Exception("Problem of connection")

        self.data_pub = rospy.Publisher('tms_db_publisher', TmsdbStamped, queue_size=10)

        self.sendDbCurrentInformation()

    def sendDbCurrentInformation(self):
        rate = rospy.Rate(100)  # 100hz

        while not rospy.is_shutdown():
            temp_dbdata = Tmsdb()
            current_environment_information = TmsdbStamped()

            # cursor = db.now.find({'$or': [{'state': 1}, {'state': 2}]})
            cursor = db.now.find()
            # print(cursor.count())
            for doc in cursor:
                del doc['_id']
                temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                current_environment_information.tmsdb.append(temp_dbdata)
            # rospy.loginfo("send db data!")
            self.data_pub.publish(current_environment_information)

            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_publisher node terminated.")
