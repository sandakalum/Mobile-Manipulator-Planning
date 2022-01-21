#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from tms_nw_api.srv import *
from flask import Flask, jsonify, abort, make_response,request
import os
import json
import requests
from multiprocessing import Process, Manager
host_url = os.getenv("ROS_HOSTNAME", "localhost")

api = Flask(__name__)
sid = 100000

def db_reader(data):
    rospy.wait_for_service('tms_db_reader')
    try:
        tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
        res = tms_db_reader(data)
        return res
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e
        return None

def tag_reader(data):
    temp_dbdata = Tmsdb()
    temp_dbdata.tag='「'+data+'」'
    target = db_reader(temp_dbdata)
    return target

def get_id_callback(req):
        if req.url in response_dic:
            res_msg = response_dic.pop(req.url)
            print res_msg
            return res_msg
        else:
            print response_dic
            return

def get_id_server():
    rospy.init_node('get_id_server')
    s = rospy.Service("get_id", get_id, get_id_callback)
    print "Ready to get_id server"
    rospy.spin()


@api.route('/get', methods=['GET'])
def get():
    response = make_response(jsonify({
        'status':'OK'
    }))
    response.statusCode = 204
    
    return response

@api.route('/post', methods=['POST'])
def post():
    if(request.headers['Content-Type']!= 'application/json'):
        print request.headers['Content-Type']
        return jsonify({
            "message":"request format is not appropriate"
        })

    print request.json
    req_word=request.json["words"]
    tms_url = request.json["url"]
    print tms_url

    print req_word
    response = search_db(req_word)
    if not response:
        payload = {
            "words":req_word,
            "uri":tms_url
        }
        ret = requests.post("http://" + host_url + ":4000/rms_svr",json = payload)
        remote_tms = ret.json()
        print remote_tms
        if remote_tms["message"] == "OK":
            response_url = remote_tms["hostname"]
            return make_response(jsonify({
            'message':'OK_nested',
            'uri':response_url
            }))

        return make_response(jsonify({
            'message':'Could not find them in tms_db',
        }))
        #else:
        #    status = 503
    res_msg = get_idResponse()
    res_msg.task_id = response[0]
    res_msg.robot_id = response[1]
    res_msg.user_id = response[2]
    res_msg.object_id = response[3]
    res_msg.place_id = response[4]

    res_msg.task_announce = response[5]
    res_msg.robot_announce = response[6]
    res_msg.user_announce = response[7]
    res_msg.object_announce = response[8]
    res_msg.place_announce = response[9]
    res_msg.data = response[10]
    
    response_dic[tms_url] = res_msg
    print response_dic
    print "OK"
    return make_response(jsonify({
            'message':'OK'
        }))

def search_db(req_words):
    task_dic ={}
    user_dic = {}
    robot_dic = {}
    object_dic = {}
    place_dic = {}
    task_id = 0
    announce = ""
    task_announce = ""
    robot_announce = ""
    user_announce = ""
    object_announce = ""
    place_announce = ""

    for word in req_words:
        res = tag_reader(word)
        if res is None:
            return False
        for target in res.tmsdb:
            if target.type == 'task':
                task_dic[target.id] = target.announce
            elif target.type == 'robot':
                robot_dic[target.id] = target.announce
            elif target.type == 'object':
                object_dic[target.id] = target.announce
            elif target.type == 'person':
                user_dic[target.id] = target.announce
            elif target.type == 'furniture':
                place_dic[target.id] = target.announce

    if len(task_dic) == 1:
        task_id = task_dic.keys()[0]
        announce = task_dic.pop(task_id)
    elif len(task_dic) > 1:
        print "len(task_dic) > 1"
        task_id = min(task_dic.keys())
        announce = task_dic.pop(task_id)
        task_dic.clear()
        #未実装

    if task_id == 8100:
        task_announce = announce
        if len(object_dic) == 1:
            object_id = object_dic.keys()[0]
            object_name = object_dic[object_id]
        elif len(object_dic) > 1:
            print "len(object_dic) > 1"
        
        place_id = 0
        place_name = ""
        temp_dbdata = Tmsdb()
        temp_dbdata.id = object_id
        temp_dbdata.state = 1

        db_target = db_reader(temp_dbdata)
        target = db_target.tmsdb[0]
        if target is None:
            return
        place_id = target.place
        temp_dbdata = Tmsdb()
        temp_dbdata.id = place_id + sid

        db_target =db_reader(temp_dbdata)
        target = db_target.tmsdb[0]
        if target is None:
            return
        place_name = target.announce

        if object_name == "" or place_name == "":
            return
        else:
            object_announce = object_name
            place_announce = place_name
            response = [task_id, 0, 0, object_id, place_id, task_announce, "", "", object_announce, place_announce,""]
            return response

    elif task_id == 8105:
        print user_dic
        task_announce = announce
        if len(user_dic) == 1:
            user_id = user_dic.keys()[0]
            user_name = user_dic[user_id]
        elif len(user_dic) > 1:
            print "len(user_dic) > 1"
            #未実装
        user_announce = user_name
        
        place_id = 0
        place_name = "" 
        temp_dbdata = Tmsdb()
        temp_dbdata.id = user_id
        temp_dbdata.state = 1

        #target = self.db_reader(temp_dbdata)
        db_target = db_reader(temp_dbdata)
        target = db_target.tmsdb[0]
        if target is None:
            return
        if target.note =="":
            return
        note = json.loads(target.note)
        rate = note["rate"]

        if rate == "":
            return
        else: 
            
            response = [task_id, 0, user_id, 0, 0, task_announce, "", user_announce, "", "",str(rate)]
            return response


    else:
        task_announce_list = announce.split(";")
        for i in range(len(task_announce_list)):
            anc_list = task_announce_list[i].split("$")
            response = ""
            announce = ""
            task_flag = 0
            robot_id = 0
            object_id = 0
            user_id = 0
            place_id = 0
            task_flag = 0
            print anc_list
            for anc in anc_list:
                if anc == "robot":
                    if len(robot_dic) == 1:
                        robot_id = robot_dic.keys()[0]
                        robot_announce = robot_dic[robot_id]
                    elif len(robot_dic) > 1:
                        print "len(robot_dic) > 1"
                                        #未実装
                    if robot_id==0:
                        if i == len(task_announce_list) - 1:
                            return False
                        else:
                            task_flag = 1
                elif anc == "object":
                    if len(object_dic) == 1:
                        object_id = object_dic.keys()[0]
                        object_announce = object_dic[object_id]
                    elif len(object_dic) > 1:
                        print "len(object_dic) > 1"
                        #未実装
                    if object_id==0:
                        if i == len(task_announce_list) - 1:
                            return False
                        else:
                            task_flag = 1
                elif anc == "user":
                    if len(user_dic) == 1:
                        user_id = user_dic.keys()[0]
                        user_announce = user_dic[user_id]
                    elif len(user_dic) > 1:
                        print "len(user_dic) > 1"
                        #未実装
                    if user_id==0:
                        if i == len(task_announce_list) - 1:
                            return False
                        else:
                            task_flag = 1
                elif anc == "place":
                    if len(place_dic) == 1:
                        place_id = place_dic.keys()[0]
                        place_announce = place_dic[place_id]
                    elif len(place_dic) > 1:
                        print "len(place_dic) > 1"
                        #未実装
                    if place_id==0:
                        if i == len(task_announce_list) - 1:
                            return False
                        else:
                            task_flag = 1
            if task_flag == 1:
                    continue

            robot_dic.clear()
            object_dic.clear()
            place_dic.clear()
            user_dic.clear()

            task_announce = task_announce_list[i]
            response = [task_id, robot_id, user_id, object_id, place_id, task_announce, robot_announce, user_announce, object_announce, place_announce,""]

            return response

def run():
    api.run(host=host_url, port = 3000)

if __name__ == "__main__":
    #api.run(host='localhost', port = 3000)
    with Manager() as manager:
        response_dic = manager.dict()
        t1 = Process(name = "http",target = run)
        t2 = Process(name = "rosservice",target = get_id_server)
        t2.start()
        t1.start()
        t2.join()
        t1.join()
    
    