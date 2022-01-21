///-----------------------------------------------------------------------------
/// @FileName sp5_cmd_vehicle.cpp
/// @Date 2013.06.04 / 2013.06.05
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include "sp5_client.h"

//------------------------------------------------------------------------------
int8_t Client::vehicleClearAlarm()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->clearAlarm();

  printf("vehicleClearAlarm result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetPower(double OnOff)
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->setPower((CORBA::Boolean)OnOff);

  if (OnOff == 1)
  {
    printf("vehicleSetPower On result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("vehicleSetPower Off result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetServo(double OnOff)
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->setServo((CORBA::Boolean)OnOff);

  if (OnOff == 1)
  {
    printf("vehicleSetServo On result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("vehicleSetServo Off result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehiclePause()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->pause();

  printf("vehiclePause result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleResume()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->resume();

  printf("vehicleResume result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleStop()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Vehicle->stop();

  printf("vehicleStop result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleGetState()
{
  if (!bInitialize)
    return CORBA_ERR;

  CORBA::Short state;
  CORBA::String_var message;

  bool ret = CommandObj_Vehicle->getState(state, message);

  printf("vehicleGetState result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  switch (state)
  {
    case 0x010:
      ret = Ready;
      printf("State: Ready\n");
      break;
    case 0x011:
      ret = Busy;
      printf("State: Busy\n");
      break;
    case 0x013:
      ret = Alarm;
      printf("State: Alarm\n");
      break;
    case 0x014:
      ret = Stuck;
      printf("State: Stuck\n");
      break;
    case 0x015:
      ret = Paused;
      printf("State: Paused\n");
      break;
    case 0x017:
      ret = Locked;
      printf("State: Locked\n");
      break;
    case 0x018:
      ret = Powered;
      printf("State: Powered\n");
      break;
    case 0x019:
      ret = Unpowered;
      printf("State: Unpowered\n");
      break;
    case 0x01A:
      ret = Caution;
      printf("State: Caution\n");
      break;
    default:
      printf("State: UnkownMessage\n");
      break;
  }

  return (int8_t)state;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleGetPos(double *x_m, double *y_m, double *theta_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm, y_mm, theta_deg;

  bool ret = CommandObj_Vehicle->getPosition(x_mm, y_mm, theta_deg);

  *x_m = x_mm / 1000;
  *y_m = y_mm / 1000;
  *theta_rad = deg2rad(theta_deg);

  // printf("vehicleGetPose result: %0.1fmm, %0.1fmm, %0.1fdeg ",*x_mm, *y_mm, *theta_deg);
  // ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetPos(double x_m, double y_m, double theta_rad)
{
  if (!bInitialize)
    return -1;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;
  double theta_deg = rad2deg(theta_rad);

  corbaD1 = (CORBA::Double)x_mm;       // Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;       // Position Y (mm)
  corbaD3 = (CORBA::Double)theta_deg;  // Theta      (deg)

  bool ret = CommandObj_Vehicle->setPosition(corbaD1, corbaD2, corbaD3);

  printf("vehicleSetPose(%0.1fmm, %0.1fmm, %0.1fdeg) result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetVel(double velT_mps, double velR_radps)
{
  if (!bInitialize)
    return CORBA_ERR;

  double velT_mmps = velT_mps * 1000;
  double velR_degps = rad2deg(velR_radps);

  // Init Value : translation = 100(mm/s),  rotation =10(deg/s)
  // Max  Value : translation = 1000(mm/s), rotation =100(deg/s)
  corbaD1 = (CORBA::Double)velT_mmps;   // Velocity of translation (mm/s)
  corbaD2 = (CORBA::Double)velR_degps;  // Velocity of rotation   (deg/s)

  bool ret = CommandObj_Vehicle->setVelocity(corbaD1, corbaD2);

  printf("vehicleSetVel(%0.1fmm/s, %0.1fdeg/s) result:", corbaD1, corbaD2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetAcc(double accT_mps2, double accR_radps2)
{
  if (!bInitialize)
    return CORBA_ERR;

  double accT_mmps2 = accT_mps2 * 1000;
  double accR_degps2 = rad2deg(accR_radps2);

  // Init Value : translation = 100(mm/s^2),  rotation =10(deg/s^2)
  // Max  Value : translation = 1000(mm/s^2), rotation =100(deg/s^2)
  corbaD1 = (CORBA::Double)accT_mmps2;   // Acceleration of translation (mm/s^2)
  corbaD2 = (CORBA::Double)accR_degps2;  // Acceleration of rotation   (deg/s^2)

  bool ret = CommandObj_Vehicle->setAcceleration(corbaD1, corbaD2);

  printf("vehicleSetAcc(%0.1fmm/s^2, %0.1fdeg/s^2) result:", corbaD1, corbaD2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveLinearAbs(double x_m, double y_m, double theta_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;
  double theta_deg = rad2deg(theta_rad);

  corbaD1 = (CORBA::Double)x_mm;       // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;       // Goal Position Y (mm)
  corbaD3 = (CORBA::Double)theta_deg;  // Goal Rotation Theta  (deg)

  bool ret = CommandObj_Vehicle->moveLinearAbs(corbaD1, corbaD2, corbaD3);

  printf("vehicleMoveLinearAbs(%0.1fmm, %0.1fmm, %0.1fdeg) result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveLinearRel(double x_m, double y_m, double theta_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;
  double theta_deg = rad2deg(theta_rad);

  corbaD1 = (CORBA::Double)x_mm;       // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;       // Goal Position Y (mm)
  corbaD3 = (CORBA::Double)theta_deg;  // Goal Rotation Theta  (deg)

  bool ret = CommandObj_Vehicle->moveLinear(corbaD1, corbaD2, corbaD3);

  printf("vehicleMoveLinearRel(%0.1fmm, %0.1fmm, %0.1fdeg) result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveCruiseAbs(double x_m, double y_m)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;

  corbaD1 = (CORBA::Double)x_mm;  // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;  // Goal Position Y (mm)

  bool ret = CommandObj_Vehicle->moveWagonAbs(corbaD1, corbaD2);

  printf("vehicleMoveCruiseAbs(%0.1fmm, %0.1fmm) result:", corbaD1, corbaD2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveCruiseRel(double x_m, double y_m)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;

  corbaD1 = (CORBA::Double)x_mm;  // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;  // Goal Position Y (mm)

  bool ret = CommandObj_Vehicle->moveWagon(corbaD1, corbaD2);

  printf("vehicleMoveCruiseRel(%0.1fmm, %0.1fmm) result:", corbaD1, corbaD2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return 1;  // OK (Success)
  else
    return -1;  // NG (Failure)
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveContinuousRel(double x_m, double y_m, double theta_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;
  double theta_deg = rad2deg(theta_rad);

  corbaD1 = (CORBA::Double)x_mm;       // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;       // Goal Position Y (mm)
  corbaD3 = (CORBA::Double)theta_deg;  // Goal Rotation Theta  (deg)

  bool ret = CommandObj_Vehicle->moveContinuous(corbaD1, corbaD2, corbaD3);

  printf("vehicleMoveContinuousRel(%0.1fmm, %0.1fmm, %0.1fdeg) result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveCircularRel(double x_m, double y_m, double angle_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double x_mm = x_m * 1000;
  double y_mm = y_m * 1000;
  double angle_deg = rad2deg(angle_rad);

  corbaD1 = (CORBA::Double)x_mm;       // Goal Position X (mm)
  corbaD2 = (CORBA::Double)y_mm;       // Goal Position Y (mm)
  corbaD3 = (CORBA::Double)angle_deg;  // Goal Rotation angle (deg)

  bool ret = CommandObj_Vehicle->moveCircular(corbaD1, corbaD2, corbaD3);

  printf("vehicleMoveCircularRel(%0.1fmm, %0.1fmm, %0.1fdeg) result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleSetJogTimeout(double timeout_msec)
{
  if (!bInitialize)
    return CORBA_ERR;

  corbaS1 = (CORBA::Short)timeout_msec;  // Time Out (msec)

  bool ret = CommandObj_Vehicle->setJogTimeout(corbaS1);

  printf("vehicleSetJogTimeout(%0.1fmsec) result:", (double)corbaS1);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::vehicleMoveJog(double vx_mps, double vy_mps, double vt_radps)
{
  if (!bInitialize)
    return CORBA_ERR;

  double vx_mmps = vx_mps * 1000;
  double vy_mmps = vy_mps * 1000;
  double vt_degps = rad2deg(vt_radps);

  corbaD1 = (CORBA::Double)vx_mmps;   // Velocity of Move X (mm/s)
  corbaD1 = (CORBA::Double)vy_mmps;   // Velocity of Move Y (mm/s)
  corbaD2 = (CORBA::Double)vt_degps;  // Velocity of Turn (deg/s)

  bool ret = CommandObj_Vehicle->moveJog(corbaD1, corbaD2, corbaD3);

  printf("vehicleMoveJog(%0.1fmm/s, %0.1fmm/s, %0.1fdeg/s) Result:", corbaD1, corbaD2, corbaD3);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
