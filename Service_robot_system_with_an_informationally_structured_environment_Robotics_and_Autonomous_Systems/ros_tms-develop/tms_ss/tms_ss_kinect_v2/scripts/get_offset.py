#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys

import rospy

import OffsetManager
import NETWORK_SETTING


if __name__ == '__main__':
    try:
        argc = len(sys.argv)
        if (argc != 2):
            print('Input camera ID for receiving data.')
            quit()
        ip_index = int(sys.argv[1]) - 1
        if (ip_index < 0 or ip_index >= NETWORK_SETTING.length):
            print('Given invalid camera ID.')
        offset = OffsetManager.OffsetManager(ip_index)
        offset.run()
    except rospy.ROSInterruptException:
        pass
