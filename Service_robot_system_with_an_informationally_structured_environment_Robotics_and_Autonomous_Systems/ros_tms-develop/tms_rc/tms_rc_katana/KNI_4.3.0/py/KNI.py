# This file was automatically generated by SWIG (http://www.swig.org).
# Version 1.3.33
#
# Don't modify this file, modify the SWIG interface instead.
# This file is compatible with both classic and new-style classes.

import _KNI
import new
new_instancemethod = new.instancemethod
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'PySwigObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static) or hasattr(self, name):
        self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError(name)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

import types
try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0
del types


ERR_NONE = _KNI.ERR_NONE
ERR_SUCCESS = _KNI.ERR_SUCCESS


class TPos(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, TPos, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, TPos, name)
    __repr__ = _swig_repr
    __swig_setmethods__["X"] = _KNI.TPos_X_set
    __swig_getmethods__["X"] = _KNI.TPos_X_get
    if _newclass:
        X = _swig_property(_KNI.TPos_X_get, _KNI.TPos_X_set)
    __swig_setmethods__["Y"] = _KNI.TPos_Y_set
    __swig_getmethods__["Y"] = _KNI.TPos_Y_get
    if _newclass:
        Y = _swig_property(_KNI.TPos_Y_get, _KNI.TPos_Y_set)
    __swig_setmethods__["Z"] = _KNI.TPos_Z_set
    __swig_getmethods__["Z"] = _KNI.TPos_Z_get
    if _newclass:
        Z = _swig_property(_KNI.TPos_Z_get, _KNI.TPos_Z_set)
    __swig_setmethods__["phi"] = _KNI.TPos_phi_set
    __swig_getmethods__["phi"] = _KNI.TPos_phi_get
    if _newclass:
        phi = _swig_property(_KNI.TPos_phi_get, _KNI.TPos_phi_set)
    __swig_setmethods__["theta"] = _KNI.TPos_theta_set
    __swig_getmethods__["theta"] = _KNI.TPos_theta_get
    if _newclass:
        theta = _swig_property(_KNI.TPos_theta_get, _KNI.TPos_theta_set)
    __swig_setmethods__["psi"] = _KNI.TPos_psi_set
    __swig_getmethods__["psi"] = _KNI.TPos_psi_get
    if _newclass:
        psi = _swig_property(_KNI.TPos_psi_get, _KNI.TPos_psi_set)

    def __init__(self, *args):
        this = _KNI.new_TPos(*args)
        try:
            self.this.append(this)
        except:
            self.this = this
    __swig_destroy__ = _KNI.delete_TPos
    __del__ = lambda self: None
TPos_swigregister = _KNI.TPos_swigregister
TPos_swigregister(TPos)
cvar = _KNI.cvar
PI = cvar.PI

PTP = _KNI.PTP
LINEAR = _KNI.LINEAR


class TMovement(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, TMovement, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, TMovement, name)
    __repr__ = _swig_repr
    __swig_setmethods__["pos"] = _KNI.TMovement_pos_set
    __swig_getmethods__["pos"] = _KNI.TMovement_pos_get
    if _newclass:
        pos = _swig_property(_KNI.TMovement_pos_get, _KNI.TMovement_pos_set)
    __swig_setmethods__["transition"] = _KNI.TMovement_transition_set
    __swig_getmethods__["transition"] = _KNI.TMovement_transition_get
    if _newclass:
        transition = _swig_property(_KNI.TMovement_transition_get, _KNI.TMovement_transition_set)
    __swig_setmethods__["velocity"] = _KNI.TMovement_velocity_set
    __swig_getmethods__["velocity"] = _KNI.TMovement_velocity_get
    if _newclass:
        velocity = _swig_property(_KNI.TMovement_velocity_get, _KNI.TMovement_velocity_set)
    __swig_setmethods__["acceleration"] = _KNI.TMovement_acceleration_set
    __swig_getmethods__["acceleration"] = _KNI.TMovement_acceleration_get
    if _newclass:
        acceleration = _swig_property(_KNI.TMovement_acceleration_get, _KNI.TMovement_acceleration_set)

    def __init__(self, *args):
        this = _KNI.new_TMovement(*args)
        try:
            self.this.append(this)
        except:
            self.this = this
    __swig_destroy__ = _KNI.delete_TMovement
    __del__ = lambda self: None
TMovement_swigregister = _KNI.TMovement_swigregister
TMovement_swigregister(TMovement)


class TCurrentMot(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, TCurrentMot, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, TCurrentMot, name)
    __repr__ = _swig_repr
    __swig_setmethods__["idx"] = _KNI.TCurrentMot_idx_set
    __swig_getmethods__["idx"] = _KNI.TCurrentMot_idx_get
    if _newclass:
        idx = _swig_property(_KNI.TCurrentMot_idx_get, _KNI.TCurrentMot_idx_set)
    __swig_setmethods__["running"] = _KNI.TCurrentMot_running_set
    __swig_getmethods__["running"] = _KNI.TCurrentMot_running_get
    if _newclass:
        running = _swig_property(_KNI.TCurrentMot_running_get, _KNI.TCurrentMot_running_set)
    __swig_setmethods__["dir"] = _KNI.TCurrentMot_dir_set
    __swig_getmethods__["dir"] = _KNI.TCurrentMot_dir_get
    if _newclass:
        dir = _swig_property(_KNI.TCurrentMot_dir_get, _KNI.TCurrentMot_dir_set)

    def __init__(self, *args):
        this = _KNI.new_TCurrentMot(*args)
        try:
            self.this.append(this)
        except:
            self.this = this
    __swig_destroy__ = _KNI.delete_TCurrentMot
    __del__ = lambda self: None
TCurrentMot_swigregister = _KNI.TCurrentMot_swigregister
TCurrentMot_swigregister(TCurrentMot)

allMotorsOff = _KNI.allMotorsOff
allMotorsOn = _KNI.allMotorsOn
calibrate = _KNI.calibrate
clearMoveBuffers = _KNI.clearMoveBuffers
closeGripper = _KNI.closeGripper
deleteMovementFromStack = _KNI.deleteMovementFromStack
deleteMovementStack = _KNI.deleteMovementStack
executeConnectedMovement = _KNI.executeConnectedMovement
executeMovement = _KNI.executeMovement
getAxisFirmwareVersion = _KNI.getAxisFirmwareVersion
getNumberOfMotors = _KNI.getNumberOfMotors
getPosition = _KNI.getPosition
getVersion = _KNI.getVersion
initKatana = _KNI.initKatana
IO_setOutput = _KNI.IO_setOutput
ModBusTCP_writeWord = _KNI.ModBusTCP_writeWord
motorOff = _KNI.motorOff
motorOn = _KNI.motorOn
moveMot = _KNI.moveMot
moveMotAndWait = _KNI.moveMotAndWait
moveToPos = _KNI.moveToPos
moveToPosEnc = _KNI.moveToPosEnc
moveToPosLin = _KNI.moveToPosLin
openGripper = _KNI.openGripper
ping = _KNI.ping
pushMovementToStack = _KNI.pushMovementToStack
runThroughMovementStack = _KNI.runThroughMovementStack
sendSplineToMotor = _KNI.sendSplineToMotor
setCollisionDetection = _KNI.setCollisionDetection
setCollisionParameters = _KNI.setCollisionParameters
setControllerParameters = _KNI.setControllerParameters
setGripper = _KNI.setGripper
setMaxAccel = _KNI.setMaxAccel
setMaxVelocity = _KNI.setMaxVelocity
setPositionCollisionLimit = _KNI.setPositionCollisionLimit
setVelocityCollisionLimit = _KNI.setVelocityCollisionLimit
startSplineMovement = _KNI.startSplineMovement
unblock = _KNI.unblock
waitForMot = _KNI.waitForMot
getDrive = _KNI.getDrive
getEncoder = _KNI.getEncoder
getVelocity = _KNI.getVelocity
IO_readInput = _KNI.IO_readInput
ModBusTCP_readWord = _KNI.ModBusTCP_readWord
