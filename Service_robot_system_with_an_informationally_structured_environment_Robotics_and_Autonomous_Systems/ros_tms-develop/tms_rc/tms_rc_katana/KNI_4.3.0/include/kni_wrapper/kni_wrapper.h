/********************************************************************************
 * kni_wrapper.h -
 * A wrapper for the kni library so that the C++ library
 * can be accessed by C based environments (MatLab, LabView ...)
 * Copyright (C) Neuronics AG
 * Philipp Keller, Tino Perucchi, 2008
********************************************************************************/

/* define EXPORT_FCNS before including this file in source files that build the library*/

#ifndef _KNI_WRAPPER_H_
#define _KNI_WRAPPER_H_

#ifdef EXPORT_FCNS
/********************************************************************************/
#include "kniBase.h"
#include "common/exception.h"
#include <vector>
/********************************************************************************/
#ifdef WIN32
/*define a macro for marking functions for dllexport*/
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif
#else
#ifdef WIN32
#define DLLEXPORT __declspec(dllimport)
#else
#define DLLEXPORT
#endif
#endif

const double PI = 3.14159265358979323846;
// additional error codes
enum
{
  ERR_NONE = 0,
  ERR_SUCCESS = 1
};

/********************************************************************************/

//!extern C because we want to access these interfaces from anywhere:
#ifdef __cplusplus
extern "C" {
#endif

//!structure representing a point & orientation in space
struct TPos
{
  //!The position in cartesian space
  double X, Y, Z;
  //!the orientation of the TCP
  double phi, theta, psi;
};
//!the transition types for a movement
enum ETransition
{
  //!Point-to-point movement
  PTP = 1,
  //!linear movement
  LINEAR = 2
};
//!structure for the
struct TMovement
{
  //!The position, see above struct TPos
  TPos pos;
  //!the transition to this position, PTP or LINEAR
  ETransition transition;
  //!The velocitiy for this particular movement
  int velocity;
  //!the acceleration for this particular movement
  int acceleration;
};
//!structure for the currently active axis
struct TCurrentMot
{
  int idx;
  bool running;
  bool dir;
};

////////////////////////////////////////////////////////////////////////////////
//!Switches all axes off
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int allMotorsOff();

//!Put all axes into hold state
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int allMotorsOn();

//!closes the Katana session
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int calibrate(int axis);

//!clears the movebuffers
DLLEXPORT int clearMoveBuffers();

//!closes the gripper if available
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int closeGripper();

//!deletes a movement from the stack
//!@param name the name of the stack
//!@param index the index of the movement to delete
DLLEXPORT int deleteMovementFromStack(char *name, int index);

//!deletes a movemnt stack
DLLEXPORT int deleteMovementStack(char *name);

//!execute a connected movement
//!@param movement	a TMovement struct to execute
//!@param startPos	a TPos struct with the start position, can be omitted if first=true
//!@param first	if this is the first of the connected movements (start at current pos)
//!@param last	if this is the last of the connected movements (wait for end of move)
DLLEXPORT int executeConnectedMovement(struct TMovement *movement, struct TPos *startPos, bool first, bool last);

//!execute a movement
//!@param movement a TMovement struct to execute, starting from the current position
DLLEXPORT int executeMovement(struct TMovement *movement);

//!flush all the movebuffers
DLLEXPORT int flushMoveBuffers();

//!gets the axis firmware version and returns it in the value argument
// length of value array at least 12, will be '\0' terminated
DLLEXPORT int getAxisFirmwareVersion(int axis, char value[]);

//!gets the pwm and returns it in the value argument
//!@param axis The axis to send the command to
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int getDrive(int axis, int &value);

//!gets the position and returns it in the value argument
//!@param axis The axis to send the command to
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int getEncoder(int axis, int &value);

//!returns the number of motors configured
DLLEXPORT int getNumberOfMotors();

//!gets a position
DLLEXPORT int getPosition(struct TPos *pos);

//!gets the velocity and returns it in the value argument
//!@param axis The axis to send the command to
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int getVelocity(int axis, int &value);

//!gets the controlboard firmware version and returns it in the value argument
// length of value array at least 8, will be '\0' terminated
DLLEXPORT int getVersion(char value[]);

//!This initializes the Katana (communication etc)
DLLEXPORT int initKatana(char *configFile, char *ipaddress);

//!reads an input from the digital I/O and returns it in the value argument
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int IO_readInput(int inputNr, int &value);

// I/O Interface
//!sets an output of the digital I/Os
//!@param ouputNr 1, or 2 for OutA or OutB
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int IO_setOutput(char output, int value);

//!reads a value from the register 'address' (if not connected, connect to the IP in katana.conf)
// and returns it in the value argument given by reference
//!@return returns -1 on failure, the read value if successful
DLLEXPORT int ModBusTCP_readWord(int address, int &value);

//!writes a value to the register 'address' (if not connected, connect to the IP in katana.conf)
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int ModBusTCP_writeWord(int address, int value);

//!Switches an axis off
//!@param axis The axis to send the command to
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int motorOff(int axis);

//!Switches an axis on
//!@param axis The axis to send the command to
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int motorOn(int axis);

//!PTP movement
//!@param axis The axis to send the command to
//!@param enc the target position
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int moveMot(int axis, int enc, int speed, int accel);

//! calls MoveMot() and WaitForMot()
//!@param axis The axis to send the command to
//!@param tolerance in encoder values (0 means wait until reached)
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!ERR_RANGE_MISMATCH if the target position is out of range
//!1 if successful
DLLEXPORT int moveMotAndWait(int axis, int targetpos, int tolerance);

//!moves in IK
DLLEXPORT int moveToPos(struct TPos *pos, int velocity, int acceleration);

//!Moves all axes to a target encoder value
//!@param enc (encX) the target positions
//!@param tolerance in encoders. sent unscaled to axis and handled there. WaitForMot (and MoveMOtAndWait) checks the
// tolerance though.
//!@param wait wait for the movement to finish
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int moveToPosEnc(int enc1, int enc2, int enc3, int enc4, int enc5, int enc6, int velocity, int acceleration,
                           int tolerance, bool _wait);

//!moves in LM
DLLEXPORT int moveToPosLin(struct TPos *targetPos, int velocity, int acceleration);

//!opens the gripper if available
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int openGripper();

//!checks the alive state of an axis
//!@param axis 0 = get all axes
//!@return If axis 0: 1 if all axes are present, negative value is the inverted number of the first axis found failing,
//!0 if no data is available. If axis != 0: 1 if heartbeat found, -1 if failed, 0 if no data is available.
DLLEXPORT int ping(int axis);

//!pushes a movement onto a stack
//!@param movement a movement structure filled with position and movement parameters
//!@param name the name of the stack to push it onto
DLLEXPORT int pushMovementToStack(struct TMovement *movement, char *name);

//!Runs through the movement stack, executes the movements
//!@param name the name of the stack to run through
//!@param loops the number of loops to run
DLLEXPORT int runThroughMovementStack(char *name, int loops);

// Linear Movement
//!sends a single polynomial to an axis (G)
//!@param axis The axis to send the command to
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!ERR_RANGE_MISMATCH if the target position is out of range
//!1 if successful
DLLEXPORT int sendSplineToMotor(int axis, int targetpos, int duration, int p0, int p1, int p2, int p3);

//!sets the collision detection on the axes.
//!@param state true = on
//!@param axis 0 = set all axes
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int setCollisionDetection(int axis, bool state);

//!sets the collision parameters
//!this function internally calls setPositionCollisionLimit and setVelocityCollisionLimit
//!@param axis 0 = set all axes
//!@param position range 1-10
//!@param velocity range 1-10
DLLEXPORT int setCollisionParameters(int axis, int position, int velocity);

//! sets the controller parameters
//!@param axis 0 = set all axes
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int setControllerParameters(int axis, int ki, int kspeed, int kpos);

//!sets or unsets whether the Katana has a Gripper
//!@param hasGripper set to true if a gripper is present. Default at startup: false
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int setGripper(bool hasGripper);

//!sets the maximum acceleration (allowed values are only 1 and 2)
//!@param axis 0 = set all axes
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int setMaxAccel(int axis, int acceleration);

//!sets the maximum velocity
//!@param axis 0 = set all axes
//!@param vel 1-180 are valid
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!1 if successful
DLLEXPORT int setMaxVelocity(int axis, int vel);

//!set the position collision limit
//!@param axis 0 = all axes
DLLEXPORT int setPositionCollisionLimit(int axis, int limit);

//!set the velocity collision limit
//!@param axis 0 = all axes
DLLEXPORT int setVelocityCollisionLimit(int axis, int limit);

//!set the force limit
//!@param axis 0 = all axes
//!@param limit limit in percent
DLLEXPORT int setForceLimit(int axis, int limit);

//!set the current force
DLLEXPORT int getForce(int axis);

//!set the current controller limit
//!@return 0 for position controller, 1 for current controller
DLLEXPORT int getCurrentControllerType(int axis);

//!starts the linear movement (G+128)
//!@return returns -1 on failure, 1 if successful
DLLEXPORT int startSplineMovement(int contd, int exactflag);

//!unblocks the robot after collision/instantstop
DLLEXPORT int unblock();

//!waits for a motor
//!@param axis The axis to wait for
//!@param targetpos (only relevant if mode is 0)
//!@param tolerance (only relevant if mode is 0) in encoder values
//!@return returns ERR_FAILURE on failure,
//!ERR_INVALID_ARGUMENT if an argument is out of range,
//!ERR_STATE_MISMATCH if the command was given to a wrong state,
//!ERR_RANGE_MISMATCH if the target position is out of range
//!1 if successful
DLLEXPORT int waitForMot(int axis, int targetpos, int tolerance);

//********************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  //_KNI_WRAPPER_H_
