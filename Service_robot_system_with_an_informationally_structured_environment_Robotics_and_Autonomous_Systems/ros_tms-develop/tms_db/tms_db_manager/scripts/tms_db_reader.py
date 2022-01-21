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
from tms_msg_db.srv import *
import tms_db_manager.tms_db_util as db_util

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb


class TmsDbReader():

    def __init__(self):
        rospy.init_node("tms_db_reader")
        rospy.on_shutdown(self.shutdown)

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        if not self.is_connected:
            raise Exception("Problem of connection")

        self.db_reader_srv = rospy.Service('tms_db_reader', TmsdbGetData, self.dbReaderSrvCallback)

    def dbReaderSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)
        temp_dbdata = Tmsdb()
        TmsdbGetData()
        result = False

        mode = 0
        temp_id = 0
        temp_type = ""
        temp_name = ""
        temp_place = 0
        sid = 100000

        MODE_ALL = 0
        MODE_NAME_IDTABLE = 1
        MODE_NAME = 2
        MODE_NAME_SENSOR = 3
        MODE_ID_IDTABLE = 4
        MODE_ID = 5
        MODE_ID_SENSOR = 6
        MODE_TYPE_IDTABLE = 7
        MODE_TYPE = 8
        MODE_TYPE_SENSOR = 9
        MODE_PLACE_IDTABLE = 10
        MODE_PLACE = 11
        MODE_PLACE_TYPE = 12
        MODE_HIERARCHY = 13
        MODE_TAG_IDTABLE = 14
        MODE_ID_STATE = 15
        MODE_ID_SENSOR_STATE = 16
        MODE_ERROR = 999

        if req.tmsdb.tag != '':
            mode = MODE_TAG_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0) and (req.tmsdb.name == ''):
            mode = MODE_ALL
        elif (req.tmsdb.id == sid) and (req.tmsdb.name != ''):
            mode = MODE_NAME_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.name != '') and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0):
            mode = MODE_NAME
        elif (req.tmsdb.id == 0) and (req.tmsdb.name != '') and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place == 0):
            mode = MODE_NAME_SENSOR
        elif (req.tmsdb.id > sid) and (req.tmsdb.type == ''):
            req.tmsdb.id -= sid
            mode = MODE_ID_IDTABLE
        elif (req.tmsdb.id != 0) and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0):
            if (req.tmsdb.state == 1):
                mode = MODE_ID_STATE
            else:
                mode = MODE_ID
        elif (req.tmsdb.id != 0) and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place == 0):
            if (req.tmsdb.state == 1):
                mode = MODE_ID_SENSOR_STATE
            else:
                mode = MODE_ID_SENSOR
        elif (req.tmsdb.id == sid) and (req.tmsdb.type != ''):
            mode = MODE_TYPE_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0):
            mode = MODE_TYPE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place == 0):
            mode = MODE_TYPE_SENSOR
        elif (req.tmsdb.id == sid) and (req.tmsdb.type == '') and (req.tmsdb.place != 0):
            mode = MODE_PLACE_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type == '') and (req.tmsdb.place != 0):
            mode = MODE_PLACE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.place != 0):
            mode = MODE_PLACE_TYPE
        elif (req.tmsdb.id != 0) and (req.tmsdb.type == '') and (req.tmsdb.place == sid):
            mode = MODE_HIERARCHY
        elif (req.tmsdb.id > 0) and ((req.tmsdb.id < 1000) or (req.tmsdb.id > 20002)):
            mode = MODE_ERROR
        else:
            mode = MODE_ERROR

        if mode == MODE_ERROR:
            temp_dbdata.note = "Wrong request! Try to check the command!"
            ret = TmsdbGetDataResponse()
            ret.tmsdb.append(temp_dbdata)
            return ret

        #  Search the ID, type, etc infomation in ID table
        if (mode == MODE_NAME) or (mode == MODE_NAME_SENSOR):
            cursor = db.default.find({'name': req.tmsdb.name})
            temp_type = cursor[0]['type']
            temp_id = cursor[0]['id']
            temp_place = cursor[0]['place']

        # Search the type, name, etc infomation in ID table
        if(mode == MODE_ID) or (mode == MODE_ID_SENSOR) or (mode == MODE_ID_STATE) or (mode == MODE_ID_SENSOR_STATE) or (mode == MODE_HIERARCHY):
            cursor = db.default.find({'id': req.tmsdb.id})
            temp_type = cursor[0]['type']
            temp_name = cursor[0]['name']
            temp_place = cursor[0]['place']

        # Search only using the place tag
        if mode == MODE_PLACE:
            ret = TmsdbGetDataResponse()
            cursor = db.now.find({'place': req.tmsdb.place})
            if cursor.count() == 0:
                temp_dbdata.note = "Wrong request! Try to check the place tag!"
                ret.tmsdb.append(temp_dbdata)
                return ret
            else:
                for doc in cursor:
                    del doc['_id']
                    temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                    ret.tmsdb.append(temp_dbdata)
                return ret

        if mode == MODE_HIERARCHY:
            loop_end_tag = False
            temp_place = req.tmsdb.id

            while temp_place != 0:
                cursor = db.now.find({'id': temp_place}).sort({'time': -1})
                if cursor.count() == 0:
                    temp_dbdata.note = "Wrong request! Try to check the target ID, place info!"
                    ret.tmsdb.append(temp_dbdata)
                    return ret
                else:
                    for doc in cursor:
                        del doc['_id']
                        temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                        ret.tmsdb.append(temp_dbdata)

                if loop_end_tag:
                    break

                temp_id = temp_dbdata.place
                cursor = db.default.find({'id': temp_id})
                temp_type = cursor[0]['type']
                temp_name = cursor[0]['name']
                temp_place = cursor[0]['place']

                if temp_id == temp_place:
                    loop_end_tag = True
                else:
                    loop_end_tag = False

                temp_place = temp_id

            return ret

        if mode == MODE_ALL:
            cursor = db.default.find()
        elif mode == MODE_TAG_IDTABLE:
            cursor = db.default.find({'tag': {'$regex': req.tmsdb.tag}})
        elif mode == MODE_NAME_IDTABLE:
            cursor = db.default.find({'name': req.tmsdb.name})
        elif mode == MODE_NAME:
            cursor = db.now.find({'name': req.tmsdb.name})
        elif mode == MODE_NAME_SENSOR:
            cursor = db.now.find({'name': req.tmsdb.name, 'sensor': req.tmsdb.sensor})
        elif mode == MODE_ID_IDTABLE:
            cursor = db.default.find({'id': req.tmsdb.id})
        elif mode == MODE_ID:
            cursor = db.now.find({'id': req.tmsdb.id})
        elif mode == MODE_ID_STATE:
            cursor = db.now.find({'id': req.tmsdb.id, 'state': 1})
        elif mode == MODE_ID_SENSOR:
            cursor = db.now.find({'id': req.tmsdb.id, 'sensor': req.tmsdb.sensor})
        elif mode == MODE_ID_SENSOR_STATE:
            cursor = db.now.find({'id': req.tmsdb.id, 'sensor': req.tmsdb.sensor, 'state': 1})
        elif mode == MODE_TYPE_IDTABLE:
            cursor = db.default.find({'type': req.tmsdb.type})
        elif mode == MODE_TYPE:
            cursor = db.now.find({'type': req.tmsdb.type})
        elif mode == MODE_TYPE_SENSOR:
            cursor = db.now.find({'sensor': req.tmsdb.sensor})
        elif mode == MODE_PLACE_IDTABLE:
            cursor = db.default.find({'place': req.tmsdb.place})
        elif mode == MODE_PLACE_TYPE:
            cursor = db.now.find({'place': req.tmsdb.place})

        ret = TmsdbGetDataResponse()
        if cursor.count() == 0:
            temp_dbdata.note = "Wrong request! Try to check the command"
            ret.tmsdb.append(temp_dbdata)
            return ret
        else:
            for doc in cursor:
                del doc['_id']
                temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                ret.tmsdb.append(temp_dbdata)
            return ret

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_reader node terminated.")
