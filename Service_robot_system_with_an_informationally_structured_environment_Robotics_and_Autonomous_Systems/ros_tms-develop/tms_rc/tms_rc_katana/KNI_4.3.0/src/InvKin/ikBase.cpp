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

#include "KNI_InvKin/ikBase.h"
#include "libKinematics.h"

CikBase::~CikBase()
{
  if (mKinematics != 0)
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    if (_kinematicsIsInitialized)
    {
      kin_clean();
    }
  }
}

void CikBase::_initKinematics()
{
  if (mKinematics == 0)
  {
    // Analytical Kinematics
    if (std::string(base->GetGNL()->modelName) == "Katana6M90A_G")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90G);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90A_F")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90T);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90B_G")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90G);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90B_F")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90T);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90G")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90G);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90T")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M90T);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M180")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics6M180);
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana5M180")
    {
      _kinematicsImpl.reset(new KNI::KatanaKinematics5M180);
    }
    else
    {
      return;
    }

    const TKatEFF* eff = base->GetEFF();
    const TKatMOT* mot = base->GetMOT();

    KNI::KatanaKinematics::metrics length;
    for (int i = 0; i < getNumberOfMotors() - 2; ++i)
    {
      length.push_back(eff->arr_segment[i]);
    }

    KNI::KinematicParameters joint;
    KNI::KatanaKinematics::parameter_container parameters;
    for (int i = 0; i < getNumberOfMotors(); ++i)
    {
      joint.epc = mot->arr[i].GetInitialParameters()->encodersPerCycle;
      joint.encOffset = mot->arr[i].GetInitialParameters()->encoderOffset;
      joint.angleOffset = mot->arr[i].GetInitialParameters()->angleOffset;
      joint.angleStop = mot->arr[i].GetInitialParameters()->angleStop;
      joint.rotDir = mot->arr[i].GetInitialParameters()->rotationDirection;
      parameters.push_back(joint);
    }

    _kinematicsImpl->init(length, parameters);
  }
  else
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    int type = -1;
    if (std::string(base->GetGNL()->modelName) == "Katana6M90A_G")
    {
      type = 1;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90A_F")
    {
      type = 0;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90B_G")
    {
      type = 4;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90B_F")
    {
      type = 3;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90G")
    {
      type = 1;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M90T")
    {
      type = 0;
    }
    else if (std::string(base->GetGNL()->modelName) == "Katana6M180")
    {
      type = 2;
    }
    else
    {
      return;
    }

    // set type
    kin_setType(type);
    int dom = kin_getDOM();
    // set link length
    FloatVector links;
    links.data[0] = (float)(base->GetEFF()->arr_segment[0] / 1000.0);
    links.data[1] = (float)(base->GetEFF()->arr_segment[1] / 1000.0);
    links.data[2] = (float)(base->GetEFF()->arr_segment[2] / 1000.0);
    links.data[3] = (float)(base->GetEFF()->arr_segment[3] / 1000.0);
    links.length = 4;
    kin_setLinkLen(&links);
    // set encoder per cycle
    IntVector epc;
    for (int i = 0; i < dom; ++i)
    {
      epc.data[i] = base->GetMOT()->arr[i].GetInitialParameters()->encodersPerCycle;
    }
    epc.length = dom;
    kin_setEPC(&epc);
    // set encoder offset
    IntVector encOffset;
    for (int i = 0; i < dom; ++i)
    {
      encOffset.data[i] = base->GetMOT()->arr[i].GetInitialParameters()->encoderOffset;
    }
    encOffset.length = dom;
    kin_setEncOff(&encOffset);
    // set rotation direction
    IntVector rotDir;
    for (int i = 0; i < dom; ++i)
    {
      if (i < 3)
      {
        // invert first three rotation directions (different angle definitions)
        rotDir.data[i] = -base->GetMOT()->arr[i].GetInitialParameters()->rotationDirection;
      }
      else
      {
        rotDir.data[i] = base->GetMOT()->arr[i].GetInitialParameters()->rotationDirection;
      }
    }
    rotDir.length = dom;
    kin_setRotDir(&rotDir);
    // set angle offset
    FloatVector angleOffset;
    for (int i = 0; i < dom; ++i)
    {
      angleOffset.data[i] = (float)(base->GetMOT()->arr[i].GetInitialParameters()->angleOffset);
    }
    angleOffset.length = dom;
    FloatVector angleOffset2;
    kin_K4D2mDHAng(&angleOffset, &angleOffset2);
    kin_setAngOff(&angleOffset2);
    // set angle range
    FloatVector angleRange;
    for (int i = 0; i < dom; ++i)
    {
      angleRange.data[i] = (float)fabs(base->GetMOT()->arr[i].GetInitialParameters()->angleRange);
    }
    angleRange.length = dom;
    kin_setAngRan(&angleRange);
    // initialize kinematics
    kin_init();
  }

  _kinematicsIsInitialized = true;
}

void CikBase::getKinematicsVersion(std::vector< int >& version)
{
  if (mKinematics == 0)
  {
    // Analytical Kinematics
    version.clear();
    version.push_back(0);
    version.push_back(1);
    version.push_back(0);
  }
  else
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    IntVector v;
    kin_getVersion(&v);
    version.clear();
    for (int i = 0; i < v.length; ++i)
    {
      version.push_back(v.data[i]);
    }
  }
}

void CikBase::setTcpOffset(double xoff, double yoff, double zoff, double psioff)
{
  if (mKinematics != 0)
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    FloatVector tcpOff;
    tcpOff.data[0] = (float)xoff;
    tcpOff.data[1] = (float)yoff;
    tcpOff.data[2] = (float)zoff;
    tcpOff.data[3] = (float)psioff;
    tcpOff.length = 4;
    kin_setTcpOff(&tcpOff);
  }
}

void CikBase::DKApos(double* position)
{
  getCoordinates(position[0], position[1], position[2], position[3], position[4], position[5]);
}

void CikBase::IKCalculate(double X, double Y, double Z, double phi, double theta, double psi,
                          std::vector< int >::iterator solution_iter)
{
  if (!_kinematicsIsInitialized)
    _initKinematics();

  if (mKinematics == 0)
  {
    // Analytical Kinematics
    std::vector< double > pose(6);
    pose[0] = X;
    pose[1] = Y;
    pose[2] = Z;
    pose[3] = phi;
    pose[4] = theta;
    pose[5] = psi;

    std::vector< int > actualPosition;
    base->recvMPS();
    for (int c = 0; c < getNumberOfMotors(); ++c)
    {
      actualPosition.push_back(getMotorEncoders(c, false));
    }

    _kinematicsImpl->IK(solution_iter, pose, actualPosition);
  }
  else
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    int maxBisection = 3;
    int nOfMot = getNumberOfMotors();

    FloatVector pose;
    pose.data[0] = (float)(X / 1000);
    pose.data[1] = (float)(Y / 1000);
    pose.data[2] = (float)(Z / 1000);
    pose.data[3] = (float)phi;
    pose.data[4] = (float)theta;
    pose.data[5] = (float)psi;
    pose.length = 6;

    IntVector actualPosition;
    base->recvMPS();
    for (int i = 0; i < nOfMot; ++i)
    {
      actualPosition.data[i] = getMotorEncoders(i, false);
    }
    actualPosition.length = nOfMot;
    FloatVector prev;
    kin_enc2rad(&actualPosition, &prev);

    FloatVector ikangle;
    kin_IK(&pose, &prev, &ikangle, maxBisection);

    IntVector ikenc;
    kin_rad2enc(&ikangle, &ikenc);

    // copy motor 6 encoder if KNI uses 6 and kinematics uses 5 motors (w/o gripper)
    if (ikenc.length == actualPosition.length - 1)
    {
      ikenc.data[ikenc.length] = actualPosition.data[ikenc.length];
      ikenc.length = actualPosition.length;
    }

    for (int i = 0; i < nOfMot; ++i)
    {
      *solution_iter = ikenc.data[i];
      solution_iter++;
    }
  }
}

void CikBase::IKCalculate(double X, double Y, double Z, double phi, double theta, double psi,
                          std::vector< int >::iterator solution_iter, const std::vector< int >& actualPosition)
{
  if (!_kinematicsIsInitialized)
    _initKinematics();

  if (mKinematics == 0)
  {
    // Analytical Kinematics
    std::vector< double > pose(6);
    pose[0] = X;
    pose[1] = Y;
    pose[2] = Z;
    pose[3] = phi;
    pose[4] = theta;
    pose[5] = psi;

    _kinematicsImpl->IK(solution_iter, pose, actualPosition);
  }
  else
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    int maxBisection = 3;
    int nOfMot = getNumberOfMotors();

    FloatVector pose;
    pose.data[0] = (float)(X / 1000);
    pose.data[1] = (float)(Y / 1000);
    pose.data[2] = (float)(Z / 1000);
    pose.data[3] = (float)phi;
    pose.data[4] = (float)theta;
    pose.data[5] = (float)psi;
    pose.length = 6;

    IntVector actPos;
    for (int i = 0; i < nOfMot; ++i)
    {
      actPos.data[i] = actualPosition.at(i);
    }
    actPos.length = nOfMot;
    FloatVector prev;
    kin_enc2rad(&actPos, &prev);

    FloatVector ikangle;
    int error = kin_IK(&pose, &prev, &ikangle, maxBisection);
    if (error)
      throw KNI::NoSolutionException();

    IntVector ikenc;
    kin_rad2enc(&ikangle, &ikenc);

    // copy motor 6 encoder if KNI uses 6 and kinematics uses 5 motors (w/o gripper)
    if (ikenc.length == actPos.length - 1)
    {
      ikenc.data[ikenc.length] = actPos.data[ikenc.length];
      ikenc.length = actPos.length;
    }

    for (int i = 0; i < nOfMot; ++i)
    {
      *solution_iter = ikenc.data[i];
      solution_iter++;
    }
  }
}

void CikBase::IKGoto(double X, double Y, double Z, double Al, double Be, double Ga, bool wait, int tolerance,
                     long timeout)
{
  if (!_kinematicsIsInitialized)
    _initKinematics();

  const TKatMOT* mot = base->GetMOT();

  std::vector< int > solution(getNumberOfMotors());
  // fills act_pos[] with the current position in degree units
  std::vector< int > act_pos(getNumberOfMotors());
  std::vector< int > distance(getNumberOfMotors());

  base->recvMPS();
  for (int idx = 0; idx < getNumberOfMotors(); ++idx)
  {
    act_pos[idx] = mot->arr[idx].GetPVP()->pos;
  }

  IKCalculate(X, Y, Z, Al, Be, Ga, solution.begin(), act_pos);
  moveRobotToEnc(solution.begin(), solution.end(), wait, tolerance, timeout);
}

void CikBase::getCoordinates(double& x, double& y, double& z, double& phi, double& theta, double& psi,
                             bool refreshEncoders)
{
  if (!_kinematicsIsInitialized)
    _initKinematics();

  if (refreshEncoders)
    base->recvMPS();

  if (mKinematics == 0)
  {
    // Analytical Kinematics
    std::vector< int > current_encoders(getNumberOfMotors());
    for (int i = 0; i < getNumberOfMotors(); i++)
    {
      current_encoders[i] = base->GetMOT()->arr[i].GetPVP()->pos;
    }

    std::vector< double > pose(6);

    _kinematicsImpl->DK(pose, current_encoders);

    x = pose[0];
    y = pose[1];
    z = pose[2];
    phi = pose[3];
    theta = pose[4];
    psi = pose[5];
  }
  else
  {
    // mKinematics == 1 and default
    // RobAnaGuess Kinematics
    int nOfMot = getNumberOfMotors();

    IntVector actPos;
    for (int i = 0; i < nOfMot; ++i)
    {
      actPos.data[i] = base->GetMOT()->arr[i].GetPVP()->pos;
    }
    actPos.length = nOfMot;
    FloatVector angle;
    kin_enc2rad(&actPos, &angle);

    FloatVector pose;
    kin_DK(&angle, &pose);

    x = pose.data[0] * 1000;
    y = pose.data[1] * 1000;
    z = pose.data[2] * 1000;
    phi = pose.data[3];
    theta = pose.data[4];
    psi = pose.data[5];
  }
}

void CikBase::getCoordinatesFromEncoders(std::vector< double >& pos, const std::vector< int >& encs)
{
  if (!_kinematicsIsInitialized)
    _initKinematics();

  if (mKinematics == 0)
  {
    // Analytical Kinematics
    _kinematicsImpl->DK(pos, encs);
  }
  else
  {
    // mKinematics == 1 and default
    int nOfMot = getNumberOfMotors();
    IntVector actPos;
    for (int i = 0; i < nOfMot; ++i)
    {
      actPos.data[i] = encs.at(i);
    }
    actPos.length = nOfMot;
    FloatVector angle;
    kin_enc2rad(&actPos, &angle);
    FloatVector pose;
    kin_DK(&angle, &pose);
    pos.clear();
    pos.push_back(pose.data[0] * 1000);
    pos.push_back(pose.data[1] * 1000);
    pos.push_back(pose.data[2] * 1000);
    pos.push_back(pose.data[3]);
    pos.push_back(pose.data[4]);
    pos.push_back(pose.data[5]);
  }
}

void CikBase::moveRobotTo(double x, double y, double z, double phi, double theta, double psi,
                          const bool waitUntilReached, const int waitTimeout)
{
  IKGoto(x, y, z, phi, theta, psi, waitUntilReached, 100, waitTimeout);
}

void CikBase::moveRobotTo(std::vector< double > coordinates, const bool waitUntilReached, const int waitTimeout)
{
  IKGoto(coordinates.at(0), coordinates.at(1), coordinates.at(2), coordinates.at(3), coordinates.at(4),
         coordinates.at(5), waitUntilReached, 100, waitTimeout);
}
