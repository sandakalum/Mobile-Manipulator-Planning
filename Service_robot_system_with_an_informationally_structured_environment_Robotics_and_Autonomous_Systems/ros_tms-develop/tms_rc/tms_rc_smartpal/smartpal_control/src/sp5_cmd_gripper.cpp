///-----------------------------------------------------------------------------
/// @FileName sp5_cmd_gripper.cpp
/// @Date 2013.06.05 / 2013.06.05
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include "sp5_client.h"

//------------------------------------------------------------------------------
int8_t Client::gripperClearAlarm(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->clearAlarms();
    printf("gripperClearAlarm R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->clearAlarms();
    printf("gripperClearAlarm L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperClearAlarm RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperSetPower(int8_t RL, double OnOff)
{
  return SUCCESS;
}

//------------------------------------------------------------------------------
int8_t Client::gripperSetServo(int8_t RL, double OnOff)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    if (OnOff == ON)
    {
      ret = CommandObj_GripperR->servoOn();
      printf("gripperSetServo on  R result: ");
      ret ? printf("Success\n") : printf("Failure\n");
    }
    else
    {
      ret = CommandObj_GripperR->servoOff();
      printf("gripperSetServo off R result: ");
      ret ? printf("Success\n") : printf("Failure\n");
    }
  }
  else if (RL == GripperL)
  {
    if (OnOff == ON)
    {
      ret = CommandObj_GripperL->servoOn();
      printf("armSetServo on  L result: ");
      ret ? printf("Success\n") : printf("Failure\n");
    }
    else
    {
      ret = CommandObj_GripperL->servoOff();
      printf("armSetServo off L result: ");
      ret ? printf("Success\n") : printf("Failure\n");
    }
  }
  else
  {
    printf("armSetServo RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperPause(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->pause();
    printf("gripperPause R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->pause();
    printf("gripperPause L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperPause RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperResume(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->resume();
    printf("armResume R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->resume();
    printf("armResume L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("armResume RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperAbort(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->abort();
    printf("gripperAbort R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->abort();
    printf("gripperAbort L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperAbort RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperStop(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->stop();
    printf("gripperStop R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->stop();
    printf("gripperStop L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperStop RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperGetState(int8_t RL)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->isMoving();
    printf("gripperIsMoving R result: ");
    ret ? printf("Moving\n") : printf("No\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->isMoving();
    printf("gripperIsMoving L result: ");
    ret ? printf("Moving\n") : printf("No\n");
  }
  else
  {
    printf("gripperIsMoving RL error\n");
    return RL_ERR;
  }

  return (int8_t)ret;
}

//------------------------------------------------------------------------------
int8_t Client::gripperGetPos(int8_t RL, double *pos_rad)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  double pos;

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->getFeedback(pos);
    // printf("gripperGetPos R result: "); ret ? printf("Success\n") : printf("Failure\n");
    // printf("gripperGetPos R: %0.1f\n",*pos);
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->getFeedback(pos);
    // printf("gripperGetPos L result: "); ret ? printf("Success\n") : printf("Failure\n");
    // printf("gripperGetPos L: %0.1f\n",*pos);
  }
  else
  {
    printf("gripperGetPos RL error\n");
    return RL_ERR;
  }

  *pos_rad = deg2rad(pos);

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t Client::gripperMoveAbs(int8_t RL, double j_rad, double vel_radps, double acc_radps2)
{
  bool ret;

  if (!bInitialize)
    return CORBA_ERR;

  double j_deg = rad2deg(j_rad);
  double vel_degps = rad2deg(vel_radps);
  double acc_degps2 = rad2deg(acc_radps2);

  // smartpal5 gripper -58 ∼ +8 degree (open direct : -)

  if (RL == GripperR)
  {
    ret = CommandObj_GripperR->move(j_deg, vel_degps, acc_degps2);
    printf("gripperMoveAbs R ( %0.1f ): ", j_deg);
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    ret = CommandObj_GripperL->move(j_deg, vel_degps, acc_degps2);
    printf("gripperMoveAbs L ( %0.1f ): ", j_deg);
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperMoveAbs RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
