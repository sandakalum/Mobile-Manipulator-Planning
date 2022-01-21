"""
The Original Code is mongodb_store package's util.py
http://www.ros.org/wiki/mongodb_store
The Initial Developers of the Original Code are Chris Burbridge and Nick Hawes.
This file is licensed under the MIT License.
This file is modified by Yoonseok Pyo.
"""

import rospy
import genpy
import pymongo
import json
import copy
from bson import json_util
from bson.objectid import ObjectId


def msg_to_document(msg):
    msg_dict = {}
    slot_types = []

    if hasattr(msg, '_slot_types'):
        slot_types = msg._slot_types
    else:
        slot_types = [None] * len(msg.__slots__)

    for (attr, type) in zip(msg.__slots__, slot_types):
        msg_dict[attr] = sanitize_value(attr, getattr(msg, attr), type)

    return msg_dict


def sanitize_value(attr, v, type):
    if isinstance(v, str):
        if type == 'uint8[]':
            v = Binary(v)
        else:
            try:
                v = unicode(v, "utf-8")
            except UnicodeDecodeError as e:
                v = Binary(v)
        return v

    if isinstance(v, rospy.Message):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Time):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Duration):
        return msg_to_document(v)
    elif isinstance(v, list):
        result = []
        for t in v:
            if hasattr(t, '_type'):
                result.append(sanitize_value(None, t, t._type))
            else:
                result.append(sanitize_value(None, t, None))
        return result
    else:
        return v


def check_connection(db_host, db_port):
    try:
        from pymongo import Connection
        Connection(db_host, db_port)
        rospy.loginfo("Connected to the ROS-TMS Database!")
        return True
    except Exception as e:
        rospy.loginfo("Error: %s" % str(e))
        rospy.loginfo("Could not connect to mongo server %s:%d" % (db_host, db_port))
        return False


def document_to_msg(document, TYPE):
    msg = TYPE()
    _fill_msg(msg, document)
    return msg


def _fill_msg(msg, dic):
    for i in dic:
        if isinstance(dic[i], dict):
            _fill_msg(getattr(msg, i), dic[i])
        else:
            setattr(msg, i, dic[i])
