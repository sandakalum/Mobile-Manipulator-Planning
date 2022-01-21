# -*- coding:utf-8 -*-

import serial
import json
import datetime
import time

dev = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
)

if __name__ == '__main__':
    print "Hello World"
    while True:
        time.sleep(1)
        cmd = "G10\r"
        dev.write(cmd)
        # print cmd
        time.sleep(0.5)
        ret = dev.read(dev.inWaiting())
        ret_array = ret.split(" ")
        # rate = ret_array[2]
        rate = ret.split(" ")[2]
        print "rate:" + rate + "     " + ret

    time = datetime.datetime.now()
    rate = 78
    note_d = {"heartrate": {"val": rate,
                            "ros_time": time.strftime("%Y/%m/%d %H:%M:%S")}
              }
    print note_d
    print note_d["heartrate"]
    print note_d["heartrate"]["val"]
    print type(note_d)
    note_j = json.dumps(note_d)
    print type(note_j)
    print note_j
    print json.dumps(note_d, indent=4)
