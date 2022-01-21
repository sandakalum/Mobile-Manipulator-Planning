/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "KNI/kmlExt.h"
#include "KNI/kmlFactories.h"

#include "common/MathHelperFunctions.h"
#include "common/Timer.h"

#include <iostream>
#include <algorithm>
#include <vector>

#define max(a, b) (((a) > (b)) ? (a) : (b))
KNI::Timer kni_timer;

/// Polling position every POLLFREQUENCY milliseconds
const int POLLFREQUENCY = 300;

void CKatana::inc(long idx, int dif, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].inc(dif, wait, tolerance, timeout);
}

void CKatana::dec(long idx, int dif, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].dec(dif, wait, tolerance, timeout);
}

void CKatana::mov(long idx, int tar, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].mov(tar, wait, tolerance, timeout);
}

void CKatana::incDegrees(long idx, double dif, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].incDegrees(dif, wait, tolerance, timeout);
}

void CKatana::decDegrees(long idx, double dif, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].decDegrees(dif, wait, tolerance, timeout);
}

void CKatana::movDegrees(long idx, double tar, bool wait, int tolerance, long timeout)
{
  base->GetMOT()->arr[idx].movDegrees(tar, wait, tolerance, timeout);
}

void CKatana::create(const char* configurationFile, CCplBase* protocol)
{
  KNI::kmlFactory infos;
  if (!infos.openFile(configurationFile))
  {
    throw ConfigFileOpenException(configurationFile);
  }
  create(&infos, protocol);
}

void CKatana::create(KNI::kmlFactory* infos, CCplBase* protocol)
{
  base->init(infos->getGNL(), infos->getMOT(), infos->getSCT(), infos->getEFF(), protocol);

  for (int i = 0; i < getNumberOfMotors(); ++i)
  {
    TMotInit init = infos->getMotInit(i);

    base->GetMOT()->arr[i].setInitialParameters(KNI_MHF::deg2rad(init.angleOffset), KNI_MHF::deg2rad(init.angleRange),
                                                init.encodersPerCycle, init.encoderOffset, init.rotationDirection);

    TMotCLB clb = infos->getMotCLB(i);
    base->GetMOT()->arr[i].setCalibrationParameters(clb.enable, clb.order, clb.dir, clb.mcf, clb.encoderPositionAfter);

    base->GetMOT()->arr[i].setDYL(infos->getMotDYL(i));
    base->GetMOT()->arr[i].setSCP(infos->getMotSCP(i));
  }
  mKatanaType = infos->getType();
  if (mKatanaType == 450)
  {
    mKinematics = infos->getKinematics();
    if (protocol != NULL)
      base->flushMoveBuffers();
  }
  else
  {
    mKinematics = 0;
  }
  if (base->checkKatanaType(mKatanaType) < 0)
  {
    // incompatible config file
    std::cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout << "Exit: Incompatible Config File!\n";
    std::cout << "Check whether you have a Katana 400 or 300 and choose the config file accordingly\n";
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n.";
    exit(0);
  }
  bool gripperIsPresent;
  int gripperOpenEncoders, gripperCloseEncoders;
  infos->getGripperParameters(gripperIsPresent, gripperOpenEncoders, gripperCloseEncoders);
  setGripperParameters(gripperIsPresent, gripperOpenEncoders, gripperCloseEncoders);
}

void CKatana::create(TKatGNL& gnl, TKatMOT& mot, TKatSCT& sct, TKatEFF& eff, CCplBase* protocol)
{
  base->init(gnl, mot, sct, eff, protocol);
}

void CKatana::calibrate()
{
  if (mKatanaType >= 400)
  {
    std::cout << "Katana4xx calibration started\n";
    // standalone calibration for Katana400
    TMotCmdFlg cflg = MCF_CALIB;
    TMotTPS tps;
    tps.tarpos = 32000;
    tps.mcfTPS = cflg;
    for (int i = 0; i < getNumberOfMotors(); ++i)
    {
      base->GetMOT()->arr[i].setCalibrated(false);
    }
    ///////////////////////////////////////
    // standalone calibration for K400:
    byte p[32];     // packet
    byte buf[256];  // readbuf
    byte sz = 10;   // readbuf size
    p[0] = 'C';
    p[1] = 0;  // calibrate all motors
    p[2] = 4;  // calibrate
    p[3] = (byte)(tps.tarpos >> 8);
    p[4] = (byte)(tps.tarpos);
    base->getProtocol()->comm(p, buf, &sz);
    ///////////////////////////////////////
    for (int i = 0; i < getNumberOfMotors(); ++i)
    {
      base->GetMOT()->arr[i].setCalibrated(true);
    }
    // wait for termination:
    p[0] = 'D';
    p[1] = 1;
    do
    {
      KNI::sleep(1000);
      base->getProtocol()->comm(p, buf, &sz);
    } while (buf[2] == 4);
    std::cout << "...done with calibration.\n";
  }
  else if (mKatanaType == 300)
  {
    std::cout << "Katana300 calibration started\n";
    KNI::sleep(500);
    //----------------------------------------------------------------//
    //"traditional" calibration, works only up to K400 V0.4.0
    // For newer Katana types, "type" has to be set to 400 in the config file section [GENERAL]
    // set motors ON before calibrating
    //----------------------------------------------------------------//
    TMotAPS aps;
    for (int i = 0; i < getNumberOfMotors(); i++)
    {
      aps.actpos = 0;
      aps.mcfAPS = MCF_ON;
      base->GetMOT()->arr[i].sendAPS(&aps);
    }
    for (int i = 0; i < getNumberOfMotors(); ++i)
    {
      for (int j = 0; j < getNumberOfMotors(); ++j)
      {
        if (base->GetMOT()->arr[j].GetCLB()->order == i)
        {
          base->GetMOT()->arr[j].setCalibrated(false);
          calibrate(j, *base->GetMOT()->arr[j].GetCLB(), *base->GetMOT()->arr[j].GetSCP(),
                    *base->GetMOT()->arr[j].GetDYL());
          base->GetMOT()->arr[j].setCalibrated(true);
          break;
        }
      }
    }
  }
}

void CKatana::calibrate(long idx, TMotCLB clb, TMotSCP scp, TMotDYL dyl)
{
  if (!clb.enable)
    return;

  searchMechStop(idx, clb.dir, scp, dyl);
  // std::cout << "setting actual position to " << base->GetMOT()->arr[idx].GetInitialParameters()->encoderOffset << "
  // with motor command flag " << clb.mcf << std::endl;
  TMotAPS aps = {clb.mcf, base->GetMOT()->arr[idx].GetInitialParameters()->encoderOffset};
  base->GetMOT()->arr[idx].sendAPS(&aps);

  mov(idx, clb.encoderPositionAfter, true);
}

void CKatana::searchMechStop(long idx, TSearchDir dir, TMotSCP _scp, TMotDYL _dyl)
{
  base->GetMOT()->arr[idx].setPwmLimits(_scp.maxppwm_nmp, _scp.maxnpwm_nmp);
  base->GetMOT()->arr[idx].setControllerParameters(_scp.kspeed_nmp, _scp.kpos_nmp, _scp.kI_nmp);
  base->GetMOT()->arr[idx].setCrashLimit(_scp.crash_limit_nmp);
  base->GetMOT()->arr[idx].setCrashLimitLinear(_scp.crash_limit_lin_nmp);
  base->GetMOT()->arr[idx].setAccelerationLimit(1);
  base->GetMOT()->arr[idx].setSpeedLimits(25, 25);

  TMotAPS aps;
  switch (dir)
  {
    case DIR_POSITIVE:
      aps.actpos = -31000;      // Set the actual position equal to the
      aps.mcfAPS = MCF_FREEZE;  // extreme opposite to direction I will move
      base->GetMOT()->arr[idx].sendAPS(&aps);
      break;
    case DIR_NEGATIVE:
      aps.actpos = 31000;       // Set the actual position equal to the
      aps.mcfAPS = MCF_FREEZE;  // extreme opposite to direction I will move
      base->GetMOT()->arr[idx].sendAPS(&aps);
      break;
  };

  TMotTPS tps;
  switch (dir)
  {
    case DIR_POSITIVE:
      tps.tarpos = 32000;
      tps.mcfTPS = MCF_ON;
      base->GetMOT()->arr[idx].sendTPS(&tps);  // Set the target position equal to
      // the extreme I am moving towards.
      break;
    case DIR_NEGATIVE:
      tps.tarpos = -32000;
      tps.mcfTPS = MCF_ON;
      base->GetMOT()->arr[idx].sendTPS(&tps);  // Set the target position equal to
      // the extreme I am moving towards.

      break;
  };

  double firstSpeedSample = 100, secondSpeedSample = 100;

  KNI::Timer poll_t(POLLFREQUENCY);
  while (true)
  {
    poll_t.Start();
    base->GetMOT()->arr[idx].recvPVP();
    firstSpeedSample = base->GetMOT()->arr[idx].GetPVP()->vel;
    // std::cout << firstSpeedSample << ", " << secondSpeedSample << std::endl;
    if ((firstSpeedSample + secondSpeedSample) == 0.0)
    {
      break;  // stopper reached
    }
    secondSpeedSample = firstSpeedSample;
    poll_t.WaitUntilElapsed();
  }
  // To avoid a compensation on the motor the actual position is set to 0
  // Otherwise, as it didn't reach the target position it could attempt to go
  // on moving
  // std::cout << "V=0 @: " << base->GetMOT()->arr[idx].GetPVP()->pos << std::endl;
  aps.actpos = 0;
  aps.mcfAPS = MCF_FREEZE;
  // std::cout << "actual pos struct set to pos 0 and motorcommandflag 8 (hold robot)\nsending actual pos..." <<
  // std::endl;
  base->GetMOT()->arr[idx].sendAPS(&aps);
  // std::cout << "sent successfully." << std::endl;

  // restore motor parameters
  base->GetMOT()->arr[idx].setPwmLimits(_scp.maxppwm_nmp, _scp.maxnpwm_nmp);
  base->GetMOT()->arr[idx].setControllerParameters(_scp.kspeed_nmp, _scp.kpos_nmp, _scp.kI_nmp);
  base->GetMOT()->arr[idx].setCrashLimit(_scp.crash_limit_nmp);
  base->GetMOT()->arr[idx].setCrashLimitLinear(_scp.crash_limit_lin_nmp);
  base->GetMOT()->arr[idx].setAccelerationLimit((short)_dyl.maxaccel_nmp);
  base->GetMOT()->arr[idx].setSpeedLimits(_dyl.maxpspeed_nmp, _dyl.maxnspeed_nmp);
}

void CKatana::setTolerance(long idx, int enc_tolerance)
{
  base->GetMOT()->arr[idx].setTolerance(enc_tolerance);
}

bool CKatana::checkENLD(long idx, double degrees)
{
  return base->GetMOT()->arr[idx].checkAngleInRange(degrees);
}

void CKatana::enableCrashLimits()
{
  base->enableCrashLimits();
}

void CKatana::disableCrashLimits()
{
  base->disableCrashLimits();
}

void CKatana::unBlock()
{
  base->unBlock();
}

void CKatana::flushMoveBuffers()
{
  base->flushMoveBuffers();
}

void CKatana::setCrashLimit(long idx, int limit)
{
  base->setCrashLimit(idx, limit);
}

////////////////////////////////////////////////////////////////
void CKatana::setPositionCollisionLimit(long idx, int limit)
{
  base->GetMOT()->arr[idx].setPositionCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////
void CKatana::setSpeedCollisionLimit(long idx, int limit)
{
  base->GetMOT()->arr[idx].setSpeedCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////
void CKatana::setForceLimit(int axis, int limit)
{
  if (axis == 0)
  {
    for (int i = 1; i <= getNumberOfMotors(); i++)
    {
      setForceLimit(i, limit);
    }
  }

  // check axis number
  if (axis < 1)
    return;
  if (axis > getNumberOfMotors())
    return;

  // can not set current limit on drive controller
  if (base->GetMOT()->arr[axis - 1].GetSFW()->type == 0)
    return;

  int force = abs(limit);
  if (force > 100)
    force = 100;

  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = axis;
  p[2] = 10;  // subcommand 10 "Set Current Controller Limit"
  p[3] = (char)(force >> 8);
  p[4] = (char)(force);
  p[5] = 0;
  base->getProtocol()->comm(p, buf, &sz);
}
////////////////////////////////////////////////////////////////
short CKatana::getForce(int axis)
{
  byte r1, r2, r3;
  base->GetMOT()->arr[axis - 1].getParameterOrLimit(244, &r1, &r2, &r3);
  char force = (char)r2;
  return (short)force;
}
////////////////////////////////////////////////////////////////
int CKatana::getCurrentControllerType(int axis)
{
  base->GetMOT()->arr[axis - 1].recvSFW();
  return (int)base->GetMOT()->arr[axis - 1].GetSFW()->type;
}
////////////////////////////////////////////////////////////////
short CKatana::getNumberOfMotors() const
{
  return base->GetMOT()->cnt;
}

int CKatana::getMotorEncoders(short number, bool refreshEncoders) const
{
  if (refreshEncoders)
    base->GetMOT()->arr[number].recvPVP();  // error handling using exceptions
  return base->GetMOT()->arr[number].GetPVP()->pos;
}

std::vector< int >::iterator CKatana::getRobotEncoders(std::vector< int >::iterator start,
                                                       std::vector< int >::const_iterator end,
                                                       bool refreshEncoders) const
{
  if (refreshEncoders)
    base->recvMPS();
  std::vector< int >::iterator iter = start;
  for (int i = 0; i < getNumberOfMotors(); ++i)
  {
    if (iter == end)
      return iter;
    (*iter) = getMotorEncoders(i, false);
    ++iter;
  }
  return iter;
}

std::vector< int > CKatana::getRobotEncoders(bool refreshEncoders) const
{
  std::vector< int > temp(getNumberOfMotors());
  getRobotEncoders(temp.begin(), temp.end(), refreshEncoders);
  return temp;
}

short CKatana::getMotorVelocityLimit(short number) const
{
  return base->GetMOT()->arr[number].GetDYL()->maxpspeed_nmp;
}
short CKatana::getMotorAccelerationLimit(short number) const
{
  return base->GetMOT()->arr[number].GetDYL()->maxaccel_nmp;
}

void CKatana::setMotorVelocityLimit(short number, short velocity)
{
  base->GetMOT()->arr[number].setSpeedLimit(velocity);
}

void CKatana::setRobotVelocityLimit(short velocity)
{
  for (short c = 0; c < getNumberOfMotors(); ++c)
  {
    base->GetMOT()->arr[c].setSpeedLimit(velocity);
  }
}

void CKatana::setMotorAccelerationLimit(short number, short acceleration)
{
  base->GetMOT()->arr[number].setAccelerationLimit(acceleration);
}

void CKatana::setRobotAccelerationLimit(short acceleration)
{
  for (short c = 0; c < getNumberOfMotors(); ++c)
  {
    base->GetMOT()->arr[c].setAccelerationLimit(acceleration);
  }
}

void CKatana::moveMotorByEnc(short number, int encoders, bool waitUntilReached, int waitTimeout)
{
  if (encoders >= 0)
  {
    inc(number, encoders, waitUntilReached, waitTimeout);
  }
  else
  {
    dec(number, abs(encoders), waitUntilReached, 100, waitTimeout);
  }
}

void CKatana::moveMotorBy(short number, double radianAngle, bool waitUntilReached, int waitTimeout)
{
  double degree = radianAngle / M_PI * 180;
  base->GetMOT()->arr[number].incDegrees(degree, waitUntilReached, 100, waitTimeout);
}

void CKatana::moveMotorToEnc(short number, int encoders, bool waitUntilReached, int encTolerance, int waitTimeout)
{
  mov(number, encoders, waitUntilReached, encTolerance, waitTimeout);
}

void CKatana::moveMotorTo(short number, double radianAngle, bool waitUntilReached, int waitTimeout)
{
  int encoders = KNI_MHF::rad2enc(radianAngle, base->GetMOT()->arr[number].GetInitialParameters()->angleOffset,
                                  base->GetMOT()->arr[number].GetInitialParameters()->encodersPerCycle,
                                  base->GetMOT()->arr[number].GetInitialParameters()->encoderOffset,
                                  base->GetMOT()->arr[number].GetInitialParameters()->rotationDirection);
  mov(number, encoders, waitUntilReached, 100, waitTimeout);
}

void CKatana::waitForMotor(short number, int encoders, int encTolerance, short mode, int waitTimeout)
{
  base->GetMOT()->arr[number].waitForMotor(encoders, encTolerance, mode, waitTimeout);
}

void CKatana::waitFor(TMotStsFlg status, int waitTimeout)
{
  base->waitFor(status, waitTimeout, _gripperIsPresent);
}

void CKatana::moveRobotToEnc(std::vector< int >::const_iterator start, std::vector< int >::const_iterator end,
                             bool waitUntilReached, int encTolerance, int waitTimeout)
{
  //         // We'll do that some other time. We need to store the velocity twice for that
  //         for(unsigned int i = 0; i < getNumberOfMotors(); ++i) {
  //             distance[i] = std::abs(act_pos[i] - solution[i]);
  //         }
  //         int maxDistNr = std::distance(distance.begin(), std::max_element(distance.begin(), distance.end()));

  int i = 0;
  std::vector< int >::const_iterator iter = start;
  while ((iter != end) && (i < getNumberOfMotors()))
  {
    //             if(i != maxDistNr) {
    //                 mot->arr[i].setSpeedLimit(distance[i]/(distance[maxDistNr]/mot->arr[maxDistNr].GetDYL()->maxpspeed_nmp));
    //             }
    mov(i, *iter, false, encTolerance, waitTimeout);
    ++i;
    ++iter;
  }

  // If wait is true, check if the target position is reached
  if (!waitUntilReached)
    return;

  const TKatMOT* mot = base->GetMOT();
  bool pos_reached;
  KNI::Timer t(waitTimeout), poll_t(POLLFREQUENCY);
  t.Start();
  while (true)
  {
    if (t.Elapsed())
      throw MotorTimeoutException();
    pos_reached = true;
    poll_t.Start();
    base->recvMPS();  // get position for all motors
    base->recvGMS();  // get status flags for all motors
    for (int idx = 0; idx < getNumberOfMotors(); idx++)
    {
      if (mot->arr[idx].GetPVP()->msf == 40)
        throw MotorCrashException();
      pos_reached &= std::abs(mot->arr[idx].GetTPS()->tarpos - mot->arr[idx].GetPVP()->pos) < 100;
    }
    if (pos_reached)
      break;
    poll_t.WaitUntilElapsed();
  }
}

void CKatana::moveRobotToEnc(std::vector< int > encoders, bool waitUntilReached, int encTolerance, int waitTimeout)
{
  moveRobotToEnc(encoders.begin(), encoders.end(), waitUntilReached, encTolerance, waitTimeout);
}

void CKatana::moveRobotToEnc4D(std::vector< int > target, int velocity, int acceleration, int encTolerance)
{
  int n, maxDistance = 0;
  short numberOfMotors = getNumberOfMotors();
  std::vector< int > diffMot, speed, oldSpeed;

  // Find the maximun difference between actual and target position for each motor
  for (n = 0; n < numberOfMotors; n++)
  {
    diffMot.push_back(std::abs(getMotorEncoders(n, true) - target.at(n)));
    maxDistance = max(diffMot.at(n), maxDistance);
  }

  // Save the old speeds and calculate the new speeds
  for (n = 0; n < numberOfMotors; n++)
  {
    oldSpeed.push_back(getMotorVelocityLimit(n));
    speed.push_back(max(static_cast< int >((static_cast< double >(diffMot.at(n)) / maxDistance) * velocity), 10));
    setMotorVelocityLimit(n, speed.at(n));
    setMotorAccelerationLimit(n, acceleration);
  }

  // Move each motor to the target position
  for (n = 0; n < numberOfMotors; n++)
  {
    moveMotorToEnc(n, target.at(n));
  }

  // Wait until the target position for all motors with the encTolerance is reached
  for (n = 0; n < numberOfMotors; n++)
  {
    waitForMotor(n, target.at(n), encTolerance);
  }

  // Restore the speeds
  for (n = 0; n < numberOfMotors; n++)
  {
    setMotorVelocityLimit(n, oldSpeed.at(n));
  }
}

void CKatana::openGripper(bool waitUntilReached, int waitTimeout)
{
  if (!_gripperIsPresent)
    return;
  moveMotorToEnc(getNumberOfMotors() - 1, _gripperOpenEncoders, waitUntilReached, waitTimeout);
}

void CKatana::closeGripper(bool waitUntilReached, int waitTimeout)
{
  if (!_gripperIsPresent)
    return;
  moveMotorToEnc(getNumberOfMotors() - 1, _gripperCloseEncoders, waitUntilReached, waitTimeout);
}

void CKatana::freezeRobot()
{
  for (int i = 0; i < getNumberOfMotors(); ++i)
    freezeMotor(i);
}
void CKatana::freezeMotor(short number)
{
  base->GetMOT()->arr[number].recvPVP();
  const TMotPVP* pvp = base->GetMOT()->arr[number].GetPVP();
  TMotTPS tps = {MCF_FREEZE, pvp->pos};
  base->GetMOT()->arr[number].sendTPS(&tps);
}
void CKatana::switchRobotOn()
{
  for (int i = 0; i < getNumberOfMotors(); ++i)
    // switchMotorOn(i); // with moving flag, old version for katana 1.1
    freezeMotor(i);  // switch on with freeze flag, new version, safer and for katana 1.2 too
}
void CKatana::switchRobotOff()
{
  for (int i = 0; i < getNumberOfMotors(); ++i)
    switchMotorOff(i);
}
void CKatana::switchMotorOn(short number)
{
  base->GetMOT()->arr[number].recvPVP();
  const TMotPVP* pvp = base->GetMOT()->arr[number].GetPVP();
  TMotTPS tps = {MCF_FREEZE, pvp->pos};
  base->GetMOT()->arr[number].sendTPS(&tps);
}
void CKatana::switchMotorOff(short number)
{
  base->GetMOT()->arr[number].recvPVP();
  const TMotPVP* pvp = base->GetMOT()->arr[number].GetPVP();
  TMotTPS tps = {MCF_OFF, pvp->pos};
  base->GetMOT()->arr[number].sendTPS(&tps);
}

void CKatana::setGripperParameters(bool isPresent, int openEncoders, int closeEncoders)
{
  _gripperIsPresent = isPresent;
  _gripperOpenEncoders = openEncoders;
  _gripperCloseEncoders = closeEncoders;
}

void CKatana::getGripperParameters(bool& isPresent, int& openEncoders, int& closeEncoders)
{
  isPresent = _gripperIsPresent;
  openEncoders = _gripperOpenEncoders;
  closeEncoders = _gripperCloseEncoders;
}

void CKatana::startSplineMovement(bool exactflag, int moreflag)
{
  int exact = 0;
  if (exactflag)
  {
    exact = 1;
  }
  if (!_gripperIsPresent)
  {
    exact += 2;
  }
  base->startSplineMovement(exact, moreflag);
}

void CKatana::sendSplineToMotor(short number, short targetPosition, short duration, short p1, short p2, short p3,
                                short p4)
{
  base->GetMOT()->arr[number].sendSpline(targetPosition, duration, p1, p2, p3, p4);
}

void CKatana::setAndStartPolyMovement(std::vector< short > polynomial, bool exactflag, int moreflag)
{
  int exact = 0;
  if (exactflag)
  {
    exact = 1;
  }
  if (!_gripperIsPresent)
  {
    exact += 2;
  }
  base->setAndStartPolyMovement(polynomial, exact, moreflag);
}

int CKatana::readDigitalIO()
{
  return base->readDigitalIO();
}
