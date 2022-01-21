#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
みまもる君を操作するGUIを表示
'''

import os
import sys
from PyQt4 import QtGui, QtCore
import argparse
import rospy
# import roslib
from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb
import tms_msg_db.srv
import tms_msg_rp.srv
import tms_msg_rc.srv

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
MAP_PATH = '/images/map.jpg'
TGT_PATH = '/images/tgt_pos.png'
WC_PATH = '/images/wc.png'

# MAP_ORIGN = QtCore.QPoint(19.0, 369.0)
MAP_ORIGN = QtCore.QPoint(0.0, 400.0)
MAP_SIZE = QtCore.QPoint(780.0, 437.0)
# ROOM_SIZE = QtCore.QPoint(16000.0, 9000.0)
ROOM_SIZE = QtCore.QPoint(15000.0, 8000.0)
ARGS = {}


class MainWidget(QtGui.QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()
        self.initUI()

    def initUI(self):
        self.tgt_pos = QtCore.QPoint(0, 0)
        self.wc_pos = QtCore.QPoint(0, 0)
        self.status = ""  # "Stopped"
        map_lbl = QtGui.QLabel(self)
        self.status_lbl = QtGui.QLabel(self)
        self.tgt_lbl = QtGui.QLabel(self)
        self.wc_lbl = QtGui.QLabel(self)
        bt_move = QtGui.QPushButton("MOVE", parent=self)
        self.bt_quit = QtGui.QPushButton("QUIT", parent=self)

        map_lbl.setGeometry(0, 0, 800, 480)
        map_lbl.setAlignment(QtCore.Qt.AlignTop)
        map_pmp = QtGui.QPixmap(SCRIPT_PATH + MAP_PATH).scaled(
            map_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
        map_lbl.setPixmap(map_pmp)
        map_lbl.mousePressEvent = self.updateTgtPos

        self.status_lbl.setText(self.status)
        # self.status_lbl.setAlignment(QtCore.Qt.AlignCenter)
        # self.status_lbl.setStyleSheet('color: red; subcontrol-position: center top; text-align: right;')
        self.status_lbl.setStyleSheet('text-align: right; color: red; font: bold 18px;')
        self.status_lbl.move(360, 465)

        self.tgt_lbl.resize(50, 50)
        tgt_pmp = QtGui.QPixmap(50, 50)
        tgt_pmp = QtGui.QPixmap(SCRIPT_PATH + TGT_PATH).scaled(
            self.tgt_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
        self.tgt_lbl.setPixmap(tgt_pmp)

        self.wc_lbl.resize(30, 30)
        wc_pmp = QtGui.QPixmap(30, 30)
        wc_pmp = QtGui.QPixmap(SCRIPT_PATH + WC_PATH).scaled(
            self.wc_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
        self.wc_lbl.setPixmap(wc_pmp)

        bt_move.move(130, 500)
        bt_move.clicked.connect(self.startMoving)

        self.bt_quit.move(530, 500)

        timer = QtCore.QTimer(parent=self)
        timer.setInterval(100)  # 100ms
        timer.timeout.connect(self.timerLoop)
        timer.start()

        self.move(10, 10)
        self.resize(800, 550)
        self.setWindowTitle("mimamorukun_gui")
        self.show()

    def timerLoop(self):
        self.updateCurPos()
        self.draw()
        # print(self.tgt_pos.x(), self.tgt_pos.y(), "      ", self.wc_pos.x(), self.wc_pos.y())
#        , "    ", self.wc_pos)
        if 200 ** 2 > (self.tgt_pos.x() - self.wc_pos.x()) ** 2 + (self.wc_pos.y() - self.wc_pos.y()) ** 2:
            self.status = ""  # "Arrived"
        self.status_lbl.setText(self.status)

    def updateCurPos(self):
        try:
            srv_client = rospy.ServiceProxy("/tms_db_reader",
                                            tms_msg_db.srv.TmsdbGetData)
            req = tms_msg_db.srv.TmsdbGetDataRequest()
            req.tmsdb.id = 2007
            req.tmsdb.sensor = 3001  # 3501  #kalman filtered data
            res = srv_client(req)
            if 0 < len(res.tmsdb):
                self.wc_pos = QtCore.QPoint(res.tmsdb[0].x * 1000, res.tmsdb[0].y * 1000)
                # print(self.wc_pos.x(), self.wc_pos.y()),
                wc_mark_pos = QtCore.QPoint(
                    self.wc_pos.x() * 1.0 / ROOM_SIZE.x() * MAP_SIZE.x() + MAP_ORIGN.x(),
                    -self.wc_pos.y() * 1.0 / ROOM_SIZE.y() * MAP_SIZE.y() + MAP_ORIGN.y())
                # print(wc_mark_pos)
                self.wc_lbl.move(wc_mark_pos - self.wc_lbl.rect().center())
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def startMoving(self):
        srv_client = None
        req = None
        if not ARGS.direct:
            srv_client = rospy.ServiceProxy("/rp_cmd", tms_msg_rp.srv.rp_cmd)
            req = tms_msg_rp.srv.rp_cmdRequest()
            req.command = 9001   # move
            req.type = True      # not sim, in Real World.But this param isn't used yet
            req.robot_id = 2007  # ID of mimamorukun
            req.arg = [-1, self.tgt_pos.x() * 0.001, self.tgt_pos.y() * 0.001, 0]
        else:
            srv_client = rospy.ServiceProxy("/mkun_goal_pose", tms_msg_rc.srv.rc_robot_control)
            req = tms_msg_rc.srv.rc_robot_controlRequest()
            req.unit = 1
            req.cmd = 15
            req.arg.append(self.tgt_pos.x() * 0.001)  # x
            req.arg.append(self.tgt_pos.y() * 0.001)  # y
            req.arg.append(0.0)                     # theta
        try:
            res = srv_client(req)
            print ("cmd result:", res.result)
            self.status = ""  # "Moving"
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def draw(self):
        pass

    def updateTgtPos(self, event):
        self.tgt_pos = QtCore.QPoint(
            (event.pos().x() * 1.0 - MAP_ORIGN.x()) / MAP_SIZE.x() * ROOM_SIZE.x(),
            (-event.pos().y() * 1.0 + MAP_ORIGN.y()) / MAP_SIZE.y() * ROOM_SIZE.y())
        tgt_mark_pos = QtCore.QPoint(event.pos().x(), event.pos().y())
        # print(event.pos())
        print self.tgt_pos
        self.tgt_lbl.move(tgt_mark_pos - self.tgt_lbl.rect().center())


def main():
    print ("\x1b[32mHello World\x1b[39m")
    global ARGS
    p = argparse.ArgumentParser(description="vicon用オブジェクトの編集（平行移動のみ）")
    p.add_argument("-d", "--direct", action='store_true', default=False, help='移動先座標を直接指定')
    ARGS = p.parse_args()
    # print args.direct

    app = QtGui.QApplication(sys.argv)
    mnw = MainWidget()
    mnw.bt_quit.clicked.connect(app.quit)

    sys.exit(app.exec_())

    # init ROS
    rospy.init_node('tms_ss_vs_viewer')
    print ("finding DB service server")
    rospy.wait_for_service('/tms_db_reader')
    print ("found DB service server")

if __name__ == '__main__':
    main()
