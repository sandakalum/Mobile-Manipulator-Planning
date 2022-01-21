#!/usr/bin/env python
# coding: utf-8
import rospy as rp
import socket
import json

from tms_msg_ss.msg import Beacon

PORT=9090
UUID="b9407f30f5f8466eaff925556b57fe6d"
MAJOR=10000
ALPHA=1#0.15#0.1

class ReceiveBeacon:
	dist = {}
	def __init__(self):
		rp.init_node('tms_ss_beacon_bridge')
		self.server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.server.bind(('',PORT))
		self.pub = rp.Publisher('tms_ss_beacon',Beacon,queue_size=10)
		print 'ready'

	def run(self):
		while not rp.is_shutdown():
			rcv,addr = self.server.recvfrom(1024)
			data = json.loads(rcv)
			if(data["uuid"]==UUID and data["major"]==MAJOR):
				distance = pow(10,float(data["measuredPower"]-data["rssi"])/20.0)
				if(distance>0.3 and distance<8.0):
					key = str(data["my_id"]) + ":" + str(data["minor"])
					if(key in self.dist):
						self.dist[key] = ALPHA * distance + (1-ALPHA) * self.dist[key]
					else:
						self.dist[key] = distance
					msg = Beacon()
					msg.pi_id = data["my_id"]
					msg.minor_id = data["minor"]
					msg.distance = round(self.dist[key],2)
					self.pub.publish(msg)
					print self.dist
					print str(data["minor"]) + " ,distance=" + str(round(self.dist[key],2)) + "m" + " ,raw=" + str(round(distance,2))

node = ReceiveBeacon()
node.run()
