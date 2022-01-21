//------------------------------------------------------------------------------
#ifndef __CLIENT_H__
#define __CLIENT_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include "idl/sp5/Mobility.hh"
#include "idl/sp5/ArmUnit.hh"
#include "idl/sp5/GripperUnit.hh"
#include "idl/sp5/LumbarUnit.hh"

#define OFF 0
#define ON 1

#define ArmR 0
#define ArmL 1

#define GripperR 0
#define GripperL 1

#define SUCCESS 1
#define FAILURE -1

#define OK 0
#define NG -1
#define STATUS_ERR -2
#define VALUE_ERR -3
#define NOT_SV_ON_ERR -4
#define FULL_MOTION_QUEUE_ERR -5
#define OVER_MAX_VEL_ERR -6
#define OVER_MAX_ACC_ERR -7
#define LOAD_ESTIMATE_ERR -8
#define FULL_COMMAND_ERR -9
#define OVER_MAX_VEL_WARNING -100
#define OVER_MAX_ACC_WARNING -101

#define CORBA_ERR -110
#define RL_ERR -111
#define SRV_UNIT_ERR -112
#define SRV_CMD_ERR -113
#define SRV_UNSUPPORTED_CMD_ERR -114

#define Unpowered 16   // 0x10
#define Powered 17     // 0x11
#define Ready 18       // 0x12
#define Busy 19        // 0x13
#define Paused 20      // 0x14
#define Alarm 21       // 0x15
#define jogBusy 22     // 0x16
#define DirectBusy 23  // 0x17
#define Locked 24      // 0x18
#define Stuck 25       // 0x19
#define Caution 26     // 0x1A

#define VehicleReady 16  // 0x10
#define VehicleBusy 17   // 0x11

#define UNIT_ALL 0
#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_GRIPPER_R 4
#define UNIT_GRIPPER_L 5
#define UNIT_LUMBA 6
#define UNIT_CC 7

#define CMD_clearAlarm 0
#define CMD_setPower 1
#define CMD_setServo 2
#define CMD_pause 3
#define CMD_resume 4
#define CMD_abort 5
#define CMD_stop 6
#define CMD_getState 7
#define CMD_getPose 8
#define CMD_move 15

using namespace std;

#define PI 3.14159265

inline double rad2deg(double rad)
{
  return (180.0 * rad / (PI));
}

inline double deg2rad(double deg)
{
  return (PI * deg / 180.0);
}

class Client
{
public:
  Client();
  virtual ~Client();
  bool Initialize();
  bool Shutdown();

  //--------------------------------------------------------------------------
  // SmartPal Vehicle
  int8_t vehicleClearAlarm();
  int8_t vehicleSetPower(double OnOff);
  int8_t vehicleSetServo(double OnOff);
  int8_t vehiclePause();
  int8_t vehicleResume();
  int8_t vehicleStop();
  int8_t vehicleGetState();
  int8_t vehicleGetPos(double *x_m, double *y_m, double *theta_rad);
  int8_t vehicleSetPos(double x_m, double y_m, double theta_rad);
  int8_t vehicleSetVel(double velT_mps, double velR_radps);
  int8_t vehicleSetAcc(double accT_mps2, double accR_radps2);
  int8_t vehicleMoveLinearAbs(double x_m, double y_m, double theta_rad);
  int8_t vehicleMoveLinearRel(double x_m, double y_m, double theta_rad);
  int8_t vehicleMoveCruiseAbs(double x_m, double y_m);
  int8_t vehicleMoveCruiseRel(double x_m, double y_m);
  int8_t vehicleMoveContinuousRel(double x_m, double y_m, double theta_rad);
  int8_t vehicleMoveCircularRel(double x_m, double y_m, double angle_rad);
  int8_t vehicleSetJogTimeout(double timeout_msec);
  int8_t vehicleMoveJog(double vx_mps, double vy_mps, double vt_radps);

  //--------------------------------------------------------------------------
  // SmartPal Arm
  int8_t armReturnValue(int8_t msg);
  int8_t armReturnAlarm(int8_t msg);
  int8_t armClearAlarm(int8_t RL);
  int8_t armSetPower(int8_t RL, double OnOff);
  int8_t armSetServo(int8_t RL, double OnOff);
  int8_t armPause(int8_t RL);
  int8_t armResume(int8_t RL);
  int8_t armAbort(int8_t RL);
  int8_t armStop(int8_t RL);
  int8_t armGetActiveAlarm(int8_t RL, u_int32_t num_request, double *return_code);
  int8_t armGetState(int8_t RL);
  int8_t armGetPos(int8_t RL, double frameID, double *posdata);
  int8_t armSetJointAcc(int8_t RL, double acc_ms);
  int8_t armSetLinearAcc(int8_t RL, double accT_ms, double accR_ms);
  bool armIsPowerOn(int8_t RL);
  bool armIsServoOn(int8_t RL);
  int8_t armMoveJointAbs(int8_t RL, double *joint_rad, double vel_radps);
  int8_t armMoveJointRel(int8_t RL, double *joint_rad, double vel_radps);
  int8_t armMoveLinearAbs(int8_t RL, double cpType, double *cartesianPos, double elbow_rad, double vt_mps,
                          double vr_radps);
  int8_t armMoveLinearRel(int8_t RL, double cpType, double *cartesianPos, double elbow_rad, double vt_mps,
                          double vr_radps);
  int8_t armGetSoftLimit(int8_t RL);

  //--------------------------------------------------------------------------
  // SmartPal Gripper
  int8_t gripperClearAlarm(int8_t RL);
  int8_t gripperSetPower(int8_t RL, double OnOff);
  int8_t gripperSetServo(int8_t RL, double OnOff);
  int8_t gripperPause(int8_t RL);
  int8_t gripperResume(int8_t RL);
  int8_t gripperAbort(int8_t RL);
  int8_t gripperStop(int8_t RL);
  int8_t gripperGetState(int8_t RL);
  int8_t gripperGetPos(int8_t RL, double *pos_rad);
  int8_t gripperMoveAbs(int8_t RL, double j_rad, double vel_radps, double acc_radps2);

  //--------------------------------------------------------------------------
  // SmartPal Lumba
  int8_t lumbaClearAlarm();
  int8_t lumbaSetPower(double OnOff);
  int8_t lumbaSetServo(double OnOff);
  int8_t lumbaPause();
  int8_t lumbaResume();
  int8_t lumbaAbort();
  int8_t lumbaStop();
  int8_t lumbaGetState();
  int8_t lumbaGetPos(double *low_rad, double *high_rad);
  int8_t lumbaMoveCooperative(double z_rad, double vel_radps, double acc_radps2);
  int8_t lumbaMove(double low_rad, double high_rad, double vel_radps, double acc_radps2);
  int8_t lumbaMoveLowerAxis(double low_rad, double vel_radps, double acc_radps2);
  int8_t lumbaMoveUpperAxis(double high_rad, double vel_radps, double acc_radps2);

protected:
  CORBA::ORB_var orb;

  YeRTUnitMobility::mobility_var CommandObj_Vehicle;

  ArmUnit_var CommandObj_ArmR;
  ArmUnit_var CommandObj_ArmL;

  GripperUnit_var CommandObj_GripperR;
  GripperUnit_var CommandObj_GripperL;

  LumbarUnit_var CommandObj_Lumbar;

  CORBA::Short corbaS1, corbaS2;
  CORBA::Double corbaD1, corbaD2, corbaD3, corbaD4, corbaD5, corbaD6;
  CORBA::String_var corbaStr1;
  CORBA::Boolean corbaB1;
  CORBA::Long corbaL1;
  CORBA::ULong corbaUL1;

  char orb_ip[24];
  char orb_port[8];
  char context_name[64];

private:
  bool bInitialize;
};
#endif  // __CLIENT_H__
