#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kouhei Kiyoyama
# URL:      https://github.com/kouhei-k
# Created:  2019-01-16

import rospy as rp
import datetime
from std_msgs.msg import Int32
import json
import pymongo  # https://api.mongodb.org/python/2.6.3/

from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb

#-------------------------------------------------------------------------
# A class to monitor user's sleep state continuously
class HeartbeatReciever:

    def __init__(self):
        rp.init_node('tms_ss_watch')
        #self.db_pub = rp.Publisher('tms_db_data', TmsdbStamped, queue_size=10)
        rp.Subscriber("watch_HBR", Int32, self.HBRrecieveCallback)

        print 'tms_ss_watch node launched'


    def HBRrecieveCallback(self, msg):
        print 'rate -> %s' % (msg)
        doc = '{ "rate":' + str(msg.data) + ' }'
        try:
            db.now.update(
                {'id': 1100},
                {
                    '$set':{
                        'note': doc
                        }
                },
                upsert = True
            )
            return True
        except:
            return False
        
        

#-------------------------------------------------------------------------



if __name__ == '__main__':
    try:
        HeartbeatReciever()
        rp.spin()
    except rp.ROSInterruptException:
        rp.loginfo("tms_ss_watch node terminated.")