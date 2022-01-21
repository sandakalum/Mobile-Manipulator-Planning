#
# Katana450 KNI Python Interface Demo
# Copyright (C) 2008 Neuronics AG
# PKE/JHA, 2008
#
"""
This Skript demonstrates the use of the KNI library through the Python wrapper
For API doc, read the kni_wrapper doc or the kni_wrapper.h file.
"""
#
import sys
sys.path.append("../")
import KNI
from KNI import TMovement
#

import KNI
KNI.initKatana("../../configfiles450/katana6M90T.cfg", "192.168.1.1")
KNI.calibrate(0)

home = TMovement()
KNI.getPosition(home.pos)
home.transition = KNI.PTP
home.velocity = 50
home.acceleration = 2

raw_input("Press [ENTER] to release robot.")
KNI.allMotorsOff()

stackname = "test"

raw_input("Press [ENTER] to store first point (p2p movement).")
m = TMovement()
KNI.getPosition(m.pos)
m.transition = KNI.PTP
m.velocity = 50
m.acceleration = 2
KNI.pushMovementToStack(m, stackname)

raw_input("Press [ENTER] to store second point (linear movement).")
m = TMovement()
KNI.getPosition(m.pos)
m.transition = KNI.LINEAR
m.velocity = 50
m.acceleration = 2
KNI.pushMovementToStack(m, stackname)

raw_input("Press [ENTER] to store third point (linear movement).")
m = TMovement()
KNI.getPosition(m.pos)
m.transition = KNI.LINEAR
m.velocity = 50
m.acceleration = 2
KNI.pushMovementToStack(m, stackname)

raw_input("Press [ENTER] to store fourth point (p2p movement).")
m = TMovement()
KNI.getPosition(m.pos)
m.transition = KNI.PTP
m.velocity = 50
m.acceleration = 2
KNI.pushMovementToStack(m, stackname)

raw_input("Press [ENTER] to store fifth and last point (p2p movement).")
m = TMovement()
KNI.getPosition(m.pos)
m.transition = KNI.PTP
m.velocity = 50
m.acceleration = 2
KNI.pushMovementToStack(m, stackname)

raw_input("Press [ENTER] to fix robot.")
KNI.allMotorsOn()

raw_input("Press [ENTER] to loop through the point list robot.")
repetitions = int(raw_input("How many repetitions? "))
KNI.runThroughMovementStack(stackname, repetitions)

KNI.executeMovement(home)

KNI.allMotorsOff()

#
