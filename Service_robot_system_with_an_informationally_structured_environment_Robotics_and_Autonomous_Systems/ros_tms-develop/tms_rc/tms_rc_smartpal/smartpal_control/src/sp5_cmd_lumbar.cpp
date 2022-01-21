///-----------------------------------------------------------------------------
/// @FileName sp5_cmd_lumbar.cpp
/// @Date 2013.06.05 / 2013.06.05
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include "sp5_client.h"

//------------------------------------------------------------------------------
int8_t Client::lumbaClearAlarm()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Lumbar->clearAlarms();

  printf("lumbaClearAlarm result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaSetPower(double OnOff)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (OnOff == ON)
  {
    ret = CommandObj_Lumbar->powerOn();
    printf("lumbaSetPower On result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    ret = CommandObj_Lumbar->powerOff();
    printf("lumbaSetPower Off result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaSetServo(double OnOff)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (OnOff == ON)
  {
    ret = CommandObj_Lumbar->servoOn();
    printf("lumbaSetServo On result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    ret = CommandObj_Lumbar->servoOff();
    printf("lumbaSetServo Off result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaPause()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Lumbar->pause();

  printf("lumbaPause result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaResume()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Lumbar->resume();

  printf("lumbaResume result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaAbort()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Lumbar->abort();

  printf("lumbaAbort result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaStop()
{
  if (!bInitialize)
    return CORBA_ERR;

  bool ret = CommandObj_Lumbar->stop();

  printf("lumbaStop result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaGetState()
{
  if (!bInitialize)
    return CORBA_ERR;

  CORBA::ULong state;
  CORBA::String_var message;

  bool ret = CommandObj_Lumbar->getState(state, message);

  printf("lumbaGetState result: ");
  ret ? printf("Success\n") : printf("Failure\n");

  switch (state)
  {
    case 0x010:
      printf("State: Unpowered\n");
      break;
    case 0x011:
      printf("State: Powered\n");
      break;
    case 0x012:
      printf("State: Ready\n");
      break;
    case 0x013:
      printf("State: Busy\n");
      break;
    case 0x014:
      printf("State: Paused\n");
      break;
    case 0x015:
      printf("State: Alarm\n");
      break;
    default:
      printf("State: UnkownMessage\n");
      break;
  }

  return (int8_t)state;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaGetPos(double *low_rad, double *high_rad)
{
  if (!bInitialize)
    return CORBA_ERR;

  double low_deg, high_deg;

  bool ret = CommandObj_Lumbar->getFeedback(low_deg, high_deg);

  // printf("lumbaGetPos result: %0.1fdeg, %0.1fdeg ",*low_deg, *high_deg);
  // ret ? printf("Success\n") : printf("Failure\n");

  *low_rad = deg2rad(low_deg);
  *high_rad = deg2rad(high_deg);

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaMoveCooperative(double z_rad, double vel_radps, double acc_radps2)
{
  if (!bInitialize)
    return CORBA_ERR;

  double z_deg = rad2deg(z_rad);
  double vel_degps = rad2deg(vel_radps);
  double acc_degps2 = rad2deg(acc_radps2);

  bool ret = CommandObj_Lumbar->moveCooperative(z_deg, vel_degps, acc_degps2);

  printf("lumbaMoveCooperative(%0.1fdeg, %0.1fdeg/s, %0.1fdeg/s^2) result:", z_deg, vel_degps, acc_degps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaMove(double low_rad, double high_rad, double vel_radps, double acc_radps2)
{
  if (!bInitialize)
    return CORBA_ERR;

  double low_deg = rad2deg(low_rad);
  double high_deg = rad2deg(high_rad);
  double vel_degps = rad2deg(vel_radps);
  double acc_degps2 = rad2deg(acc_radps2);

  bool ret = CommandObj_Lumbar->move(low_deg, high_deg, vel_degps, acc_degps2);

  printf("lumbaMove(%0.1fdeg, %0.1fdeg, %0.1fdeg/s, %0.1fdeg/s^2) result:", low_deg, high_deg, vel_degps, acc_degps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaMoveLowerAxis(double low_rad, double vel_radps, double acc_radps2)
{
  if (!bInitialize)
    return CORBA_ERR;

  double low_deg = rad2deg(low_rad);
  double vel_degps = rad2deg(vel_radps);
  double acc_degps2 = rad2deg(acc_radps2);

  bool ret = CommandObj_Lumbar->moveLowerAxis(low_deg, vel_degps, acc_degps2);

  printf("lumbaMoveLowerAxis(%0.1fdeg, %0.1fdeg/s, %0.1fdeg/s^2) result:", low_deg, vel_degps, acc_degps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::lumbaMoveUpperAxis(double high_rad, double vel_radps, double acc_radps2)
{
  if (!bInitialize)
    return CORBA_ERR;

  double high_deg = rad2deg(high_rad);
  double vel_degps = rad2deg(vel_radps);
  double acc_degps2 = rad2deg(acc_radps2);

  bool ret = CommandObj_Lumbar->moveUpperAxis(high_deg, vel_degps, acc_degps2);

  printf("lumbaMoveUpperAxis(%0.1fdeg, %0.1fdeg/s, %0.1fdeg/s^2) result:", high_deg, vel_degps, acc_degps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
