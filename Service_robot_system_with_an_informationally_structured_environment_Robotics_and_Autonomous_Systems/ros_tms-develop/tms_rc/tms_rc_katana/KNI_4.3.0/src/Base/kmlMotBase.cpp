//
// C++ Implementation: kmlMotBase
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "KNI/kmlMotBase.h"

#include "common/MathHelperFunctions.h"
#include "common/Timer.h"
#include <iostream>

bool CMotBase::init(CKatBase* _own, const TMotDesc _motDesc, CCplBase* _protocol)
{
  gnl.own = _own;
  gnl.SID = _motDesc.slvID;
  protocol = _protocol;
  try
  {
    if (protocol != NULL)
      recvSFW();
  }
  catch (ParameterReadingException pre)
  {
    sfw.type = 0;  // set position controller for now
    return true;
  }
  return true;
}

void CMotBase::resetBlocked()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  recvPVP();

  p[0] = 'C';
  p[1] = gnl.SID;
  p[2] = MCF_FREEZE;  // Flag to freeze
  p[3] = (byte)(GetPVP()->pos >> 8);
  p[4] = (byte)(GetPVP()->pos);

  protocol->comm(p, buf, &sz);

  aps.mcfAPS = MCF_FREEZE;
}

void CMotBase::sendAPS(const TMotAPS* _aps)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  p[0] = 'C';
  p[1] = gnl.SID + 128;
  p[2] = _aps->mcfAPS;
  p[3] = (byte)(_aps->actpos >> 8);
  p[4] = (byte)(_aps->actpos);

  protocol->comm(p, buf, &sz);

  if (!buf[1])
    throw ParameterWritingException("APS");

  aps = *_aps;
}

void CMotBase::sendTPS(const TMotTPS* _tps)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  p[0] = 'C';
  p[1] = gnl.SID;
  p[2] = _tps->mcfTPS;
  p[3] = (byte)(_tps->tarpos >> 8);
  p[4] = (byte)(_tps->tarpos);

  protocol->comm(p, buf, &sz);
  if (!buf[1])
    throw ParameterWritingException("TPS");
  tps = *_tps;
}

void CMotBase::recvPVP()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  p[0] = 'D';
  p[1] = gnl.SID;

  protocol->comm(p, buf, &sz);
  if (!buf[1])
    throw ParameterReadingException("PVP");
  pvp.msf = (TMotStsFlg)buf[2];
  pvp.pos = (((short)buf[3]) << 8) | buf[4];
  pvp.vel = (((short)buf[5]) << 8) | buf[6];
  pvp.pwm = buf[7];
}

void CMotBase::recvSFW()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  p[0] = 'V';
  p[1] = gnl.SID;
  p[2] = 32;

  protocol->comm(p, buf, &sz);
  if (!buf[1])
    throw ParameterReadingException("SFW");
  sfw.version = buf[3];
  sfw.subversion = buf[4];
  sfw.revision = buf[5];
  sfw.type = buf[6];
  sfw.subtype = buf[7];
}

void CMotBase::setSpeedLimits(short positiveVelocity, short negativeVelocity)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 3;  // subcommand 3 "Set Speed Limits"
  p[3] = static_cast< byte >(positiveVelocity);
  p[4] = static_cast< byte >(negativeVelocity);
  p[5] = 0;

  protocol->comm(p, buf, &sz);

  dyl.maxnspeed = dyl.maxnspeed_nmp = negativeVelocity;
  dyl.maxpspeed = dyl.maxpspeed_nmp = positiveVelocity;
}

void CMotBase::setAccelerationLimit(short acceleration)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 4;  // subcommand 4 "Set Acceleration Limit"
  p[3] = static_cast< byte >(acceleration);
  p[4] = 0;
  p[5] = 0;

  protocol->comm(p, buf, &sz);
  dyl.maxaccel_nmp = dyl.maxaccel = static_cast< byte >(acceleration);
}

void CMotBase::setPwmLimits(byte maxppwm, byte maxnpwm)
{
  if (sfw.type == 1)
    return;  // can not set pwm limit on current controller

  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 2;  // subcommand 2 "Set PWM Limits"
  p[3] = maxppwm;
  p[4] = maxnpwm;
  p[5] = 0;

  protocol->comm(p, buf, &sz);
  scp.maxppwm_nmp = scp.maxppwm = maxppwm;
  scp.maxnpwm_nmp = scp.maxnpwm = maxnpwm;

  return;
}

void CMotBase::setControllerParameters(byte kSpeed, byte kPos, byte kI)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 1;  // subcommand 1 "Set Controller Parameters"
  p[3] = kSpeed;
  p[4] = kPos;
  p[5] = kI;

  protocol->comm(p, buf, &sz);
  scp.kspeed_nmp = scp.kP_speed = kSpeed;
  scp.kpos_nmp = scp.kP = kPos;
  scp.kI_nmp = kI;  // no corresponding old motor parameter

  return;
}

void CMotBase::setCrashLimit(int limit)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 5;  // subcommand 5 "Set Crash Limit"
  p[3] = (byte)(limit >> 8);
  p[4] = (byte)limit;
  p[5] = 0;

  protocol->comm(p, buf, &sz);
  scp.crash_limit_nmp = limit;

  return;
}

void CMotBase::setCrashLimitLinear(int limit_lin)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 6;  // subcommand 6 "Set Crash Limit Linear"
  p[3] = (byte)(limit_lin >> 8);
  p[4] = (byte)limit_lin;
  p[5] = 0;

  protocol->comm(p, buf, &sz);
  scp.crash_limit_lin_nmp = limit_lin;

  return;
}
///////////////////////////////////////////////////////////////
// for Katana400s:
void CMotBase::setSpeedCollisionLimit(int limit)
{
  byte p[32];
  byte buf[256];
  byte sz = 0;
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 7;
  p[3] = (byte)limit;  // for both the linear and non-linear the same:
  p[4] = (byte)limit;
  p[5] = 0;
  protocol->comm(p, buf, &sz);
  scp.crash_limit_nmp = limit;
  return;
}
///////////////////////////////////////////////////////////////
// for Katana400s:
void CMotBase::setPositionCollisionLimit(int limit)
{
  byte p[32];
  byte buf[256];
  byte sz = 0;
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = 5;
  p[3] = (byte)(limit >> 8);
  p[4] = (byte)limit;
  p[5] = 0;
  protocol->comm(p, buf, &sz);
  scp.crash_limit_nmp = limit;
  return;
}

void CMotBase::getParameterOrLimit(int subcommand, byte* R1, byte* R2, byte* R3)
{
  // illegal subcommand
  if ((subcommand > 255) || (subcommand < 240))
  {
    *R1 = 0;
    *R2 = 0;
    *R3 = 0;
    return;
  }
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = gnl.SID;
  p[2] = (byte)subcommand;
  p[3] = 0;
  p[4] = 0;
  p[5] = 0;
  protocol->comm(p, buf, &sz);
  *R1 = buf[3];
  *R2 = buf[4];
  *R3 = buf[5];

  return;
}

void CMotBase::sendSpline(short targetPosition, short duration, short p1, short p2, short p3, short p4)
{
  std::vector< byte > sendBuf(14), recvBuf(2, 0);
  byte readBytes = 0;

  sendBuf[0] = 'G';
  sendBuf[1] = gnl.SID;
  sendBuf[2] = static_cast< byte >(targetPosition >> 8);
  sendBuf[3] = static_cast< byte >(targetPosition);
  sendBuf[4] = static_cast< byte >(duration >> 8);
  sendBuf[5] = static_cast< byte >(duration);

  sendBuf[6] = static_cast< byte >(p1 >> 8);
  sendBuf[7] = static_cast< byte >(p1);
  sendBuf[8] = static_cast< byte >(p2 >> 8);
  sendBuf[9] = static_cast< byte >(p2);
  sendBuf[10] = static_cast< byte >(p3 >> 8);
  sendBuf[11] = static_cast< byte >(p3);
  sendBuf[12] = static_cast< byte >(p4 >> 8);
  sendBuf[13] = static_cast< byte >(p4);

  protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}

void CMotBase::setInitialParameters(double angleOffset, double angleRange, int encodersPerCycle, int encoderOffset,
                                    int rotationDirection)
{
  _initialParameters.angleOffset = angleOffset;
  _initialParameters.angleRange = angleRange;
  _initialParameters.encoderOffset = encoderOffset;
  _initialParameters.encodersPerCycle = encodersPerCycle;
  _initialParameters.rotationDirection = rotationDirection;

  _initialParameters.angleStop = angleOffset + angleRange;

  int encoderStop =
      encoderOffset - rotationDirection * static_cast< int >(encodersPerCycle * (angleRange / (2.0 * M_PI)));

  _encoderLimits.enc_minpos = (encoderOffset > encoderStop) ? encoderStop : encoderOffset;
  _encoderLimits.enc_maxpos = (encoderOffset < encoderStop) ? encoderStop : encoderOffset;
  _encoderLimits.enc_per_cycle = encodersPerCycle;
  _encoderLimits.enc_range = std::abs(_encoderLimits.enc_minpos - _encoderLimits.enc_maxpos);
}

void CMotBase::setCalibrationParameters(bool doCalibration, short order, TSearchDir direction,
                                        TMotCmdFlg motorFlagAfter, int encoderPositionAfter)
{
  _calibrationParameters.enable = doCalibration;
  _calibrationParameters.order = order;
  _calibrationParameters.dir = direction;
  _calibrationParameters.mcf = motorFlagAfter;
  _calibrationParameters.encoderPositionAfter = encoderPositionAfter;
  _calibrationParameters.isCalibrated = false;
}

void CMotBase::setCalibrated(bool calibrated)
{
  _calibrationParameters.isCalibrated = calibrated;
}

void CMotBase::setTolerance(int tolerance)
{
  _encoderLimits.enc_tolerance = tolerance;
}

bool CMotBase::checkAngleInRange(double angle)
{
  return (angle >= _initialParameters.angleOffset) && (angle <= _initialParameters.angleStop);
}
bool CMotBase::checkEncoderInRange(int encoder)
{
  return (encoder >= _encoderLimits.enc_minpos) && (encoder <= _encoderLimits.enc_maxpos);
}

void CMotBase::inc(int dif, bool wait, int tolerance, long timeout)
{
  recvPVP();
  mov(GetPVP()->pos + dif, wait, tolerance, timeout);
}

void CMotBase::dec(int dif, bool wait, int tolerance, long timeout)
{
  recvPVP();
  mov(GetPVP()->pos - dif, wait, tolerance, timeout);
}

void CMotBase::mov(int tar, bool wait, int tolerance, long timeout)
{
  if (!checkEncoderInRange(tar))
    throw MotorOutOfRangeException();

  tps.mcfTPS = MCF_ON;
  tps.tarpos = tar;

  sendTPS(&tps);

  if (wait)
    waitForMotor(tar, tolerance, 0, timeout);
  else
    return;
}

void CMotBase::waitForMotor(int target, int encTolerance, short mode, int waitTimeout)
{
  const long POLLFREQUENCY = 200;
  KNI::Timer t(waitTimeout), poll_t(POLLFREQUENCY);
  t.Start();
  while (true)
  {
    if (t.Elapsed())
      throw MotorTimeoutException();
    poll_t.Start();
    recvPVP();
    if (GetPVP()->msf == 40)
      throw MotorCrashException();
    switch (mode)
    {
      case 0:
        if (std::abs(target - GetPVP()->pos) < encTolerance)
          return;  // position reached
        break;
      case 1:
        if (GetPVP()->msf == MSF_DESPOS)
          return;  // non-linear movement reached
        break;
      case 2:
        if (GetPVP()->msf == MSF_NLINMOV)
          return;  // linear movement reached
        break;
    }
    poll_t.WaitUntilElapsed();
  }
}

void CMotBase::incDegrees(double dif, bool wait, int tolerance, long timeout)
{
  int dir;
  _initialParameters.rotationDirection == DIR_NEGATIVE ? dir = 1 : dir = -1;
  int enc = (int)(dif / 360 * dir * (double)_initialParameters.encodersPerCycle);
  inc(enc, wait, tolerance, timeout);
}

void CMotBase::decDegrees(double dif, bool wait, int tolerance, long timeout)
{
  int dir;
  _initialParameters.rotationDirection == DIR_NEGATIVE ? dir = 1 : dir = -1;
  int enc = (int)(dif / 360 * dir * (double)_initialParameters.encodersPerCycle);
  dec(enc, wait, tolerance, timeout);
}

void CMotBase::movDegrees(double tar, bool wait, int tolerance, long timeout)
{
  int enc = KNI_MHF::rad2enc(KNI_MHF::deg2rad(tar), _initialParameters.angleOffset, _initialParameters.encodersPerCycle,
                             _initialParameters.encoderOffset, _initialParameters.rotationDirection);
  mov(enc, wait, tolerance, timeout);
}
