#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import time
import rospy

from ninebot.srv import nav_command

def usage():
    print "[namespace command x y a] command = start, goal, cancel"
    sys.exit(1)

def send_command(name, com, x, y, a):
    try:
        rospy.wait_for_service(name + '/ninebot_command', 10)
        ninebot_command = rospy.ServiceProxy(name + '/ninebot_command', nav_command)
        response = ninebot_command(com, x, y, a)
        print response.result
    except:
        print "ninebot_command service call failed"
        sys.exit(1)

def main(args):

    send_x = send_y = send_a = float()
    
    try: #syntax analysis
        if len(args) != 3 and len(args) != 6: 
            sys.exit(1) #go to except
        namespace = str(args[1])
        command   = str(args[2])
        if command == 'start' or command == 'goal':
            send_x    = float(args[3])
            send_y    = float(args[4])
            send_a    = float(args[5])
        elif command != 'cancel': 
            sys.exit(1) #go to except
    except: usage()

    send_command(namespace, command, send_x, send_y, send_a)

if __name__ == '__main__':
  try:
    main(sys.argv)
  except rospy.ROSInterruptException: pass