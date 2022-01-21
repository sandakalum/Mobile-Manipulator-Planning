#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
DBに保存されているみまもる君の座標を表示
'''


import rospy
import tms_msg_db.srv

import datetime


def main():
    print ("\x1b[32mHello World\x1b[39m")
    rospy.init_node('dataPlot')
    print ("finding DB service server")
    rospy.wait_for_service('/tms_db_reader/dbreader')
    print ("found DB service server")
    r = rospy.Rate(4)
    while True:
        try:
            srv_client = rospy.ServiceProxy("/tms_db_reader/dbreader",
                                            tms_msg_db.srv.TmsdbGetData)
            req = tms_msg_db.srv.TmsdbGetDataRequest()
            req.tmsdb.id = 2007
            req.tmsdb.sensor = 3001  # 3501
            res = srv_client(req)
            if 0 < len(res.tmsdb):
                print datetime.datetime.now(),
                print"x:%4f  y:%4f z:%4f" % (res.tmsdb[0].x, res.tmsdb[0].y, res.tmsdb[0].ry)
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)
        r.sleep()
    pass


if __name__ == '__main__':
    main()
