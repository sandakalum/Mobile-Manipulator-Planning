#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_listener.msg import julius_msg
from tms_ur_listener.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import sys
import socket
import requests
import re
import subprocess
import shlex
import time
import json
import base64
import threading


julius_path = '/usr/local/bin/julius'
jconf_path = '/home/pi/dictation-kit-v4.3.1-linux/tms.jconf'
julius = None
julius_socket = None
adinrec_path = '/usr/local/bin/adinrec'
wav_file = '/home/pi/catkin_ws/src/tms_ur_listener/script/rec.wav'
gs_filename = 'gs://ros-tms/rec.wav'

def invoke_julius():
    print 'INFO : invoke julius'
    args = julius_path + ' -C ' + jconf_path + ' -module '
    p = subprocess.Popen(
            shlex.split(args),
            stdin=None,
            stdout=None,
            stderr=None
        )
    print 'INFO : invoke julius complete.'
    print 'INFO : wait 2 seconds.'
    time.sleep(3.0)
    print 'INFO : invoke julius complete'
    return p


def kill_julius(julius):
    print 'INFO : terminate julius'
    julius.kill()
    while julius.poll() is None:
        print 'INFO : wait for 0.1 sec julius\' termination'
        time.sleep(0.1)
    print 'INFO : terminate julius complete'


def get_OS_PID(process):
    psef = 'ps -ef | grep ' + process + ' | grep -ve grep -vie python |head -1|awk \'{print($2)}\''
    if sys.version_info.major == 3:
        PID = str(subprocess.check_output(psef, shell=True), encoding='utf-8').rstrip ()
    else:
        PID = subprocess.check_output(psef, shell=True).rstrip ()
    return PID


def create_socket():
    print 'INFO : create a socket to connect julius'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 10500))
    print 'INFO : create a socket to connect julius complete'
    return s


def delete_socket(s):
    print 'INFO : delete a socket'
    s.close()
    print 'INFO : delete a socket complete'
    return True


def invoke_julius_set():
    julius = invoke_julius()
    julius_socket = create_socket()
    sf = julius_socket.makefile('rb')
    return (julius, julius_socket, sf)


def power_callback(data):
    rospy.loginfo(data)
    global julius,julius_socket,sf
    if data.data == True:
        rospy.loginfo("invoke julius")
        julius, julius_socket, sf = invoke_julius_set()
    else:
        rospy.loginfo("DIE julius")
        kill_julius(julius)
        delete_socket(julius_socket)

def gSpeech_callback(req):
    print "listen command"
    args = adinrec_path + ' ' + wav_file
    ret = subprocess.check_output(shlex.split(args))
    speak = String()
    speak.data = "\sound2"
    speaker_pub.publish(speak)
    print "recording succeed"

    file = open(wav_file,'rt').read()
    enc = base64.b64encode(file)
    print "enc success"

    data = "{'config':{'encoding':'LINEAR16','sampleRate':16000,'languageCode':'ja-JP'},'audio':{'content':'" + enc + "'}}"
    json_file = open('/home/pi/catkin_ws/src/tms_ur_listener/script/sync-request.json','w')
    json_file.write(data)
    json_file.close()
    print "write json_file"

    args = 'curl -s -k -H "Content-Type: application/json" -H "Authorization: Bearer '+token.rstrip('\n')+'" https://speech.googleapis.com/v1beta1/speech:syncrecognize -d @/home/pi/catkin_ws/src/tms_ur_listener/script/sync-request.json'
    print args
    ret = subprocess.check_output(shlex.split(args))
    print ret
    json_dict = json.loads(ret,"utf-8")
    if "results" in json_dict:
        script = json_dict["results"][0]["alternatives"][0]["transcript"]
        val = float(json_dict["results"][0]["alternatives"][0]["confidence"])
    else:
        script = ""
        val = 0.0

    resp = gSpeech_msgResponse()
    resp.data = script
    resp.value = val
    return resp

    # msg = julius_msg()
    # msg.data = script
    # msg.value = val
    # pub.publish(msg)

def get_token():
    global token
    args = 'gcloud auth print-access-token'
    token = subprocess.check_output(shlex.split(args))
    print token
    t=threading.Timer(600,get_token)
    t.setDaemon(True)
    t.start()

def main():
    rospy.init_node("tms_ur_listener_client",anonymous=True)
    global speaker_pub,pub
    speaker_pub = rospy.Publisher("/speaker",String,queue_size=10)
    pub = rospy.Publisher('julius_msg',julius_msg,queue_size=10)
    rate = rospy.Rate(100)
    rospy.Subscriber("/julius_power",Bool,power_callback)
    # rospy.Subscriber("gSpeech",Empty,gSpeech_callback)
    rospy.Service("gSpeech",gSpeech_msg,gSpeech_callback)

    global julius,julius_socket,sf
    julius, julius_socket, sf = invoke_julius_set()

    reResult = re.compile(u'WHYPO WORD="(\S*)" .* CM="(\d\.\d*)"')

    t=threading.Thread(target=get_token)
    t.setDaemon(True)
    t.start()

    while not rospy.is_shutdown():
        if julius.poll() is None:
            try:
                line = sf.readline().decode('utf-8')
            except socket.timeout:
                line = ""
                continue;
            print line
            tmp = reResult.search(line)
            if tmp:
                print tmp.group(1)
                msg = julius_msg()
                msg.data = tmp.group(1)
                msg.value = float(tmp.group(2))
                pub.publish(msg)
        rate.sleep()
    rospy.loginfo("exit")
    kill_julius(julius)
    delete_socket(julius_socket)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")
        kill_julius(julius)
        delete_socket(julius_socket)
        sys.exit(0)
