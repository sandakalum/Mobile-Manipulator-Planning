#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_speaker.srv import *
from std_msgs.msg import String
import subprocess

def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    quality=['-a','0.57']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+quality+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t)
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)

def speak(data):
    if data == '':
        return 0
    elif data[0]=='\\':
        aplay = ['aplay','-q','/home/pi/catkin_ws/src/tms_ur_speaker/script/'+data[1:]+'.wav']
        wr = subprocess.Popen(aplay)
        soxi = ['soxi','-D','/home/pi/catkin_ws/src/tms_ur_speaker/script/'+data[1:]+'.wav']
        ret = subprocess.check_output(soxi)
        print ret
        return ret
    else:
        talk = data.replace(',','')
        jtalk(talk)
        soxi = ['soxi','-D','open_jtalk.wav']
        ret = subprocess.check_output(soxi)
        print ret
        return ret

def callback(data):
    print data.data
    speak(data.data)

def callback_srv(req):
    print req.data
    ret = speak(req.data)
    return speaker_srvResponse(float(ret))

def main():
    rospy.init_node("tms_ur_speaker",anonymous=True)
    rospy.Subscriber("speaker",String,callback)
    rospy.Service('speaker_srv',speaker_srv,callback_srv)
    rospy.loginfo("ready")
    rospy.spin()
    rospy.loginfo("exit")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException");
        sys.exit(0)
