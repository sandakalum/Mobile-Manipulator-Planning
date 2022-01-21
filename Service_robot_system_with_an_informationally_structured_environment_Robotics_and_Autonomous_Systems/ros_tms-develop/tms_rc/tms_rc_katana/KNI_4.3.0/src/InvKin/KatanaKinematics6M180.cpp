/***************************************************************************
 *   Copyright (C) 2006 by Tiziano Mueller                                 *
 *   tiziano.mueller@neuronics.ch                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "KNI_InvKin/KatanaKinematics6M180.h"
#include "common/MathHelperFunctions.h"
#include <algorithm>

namespace KNI
{
const double KatanaKinematics6M180::_tolerance = 0.001;
const int KatanaKinematics6M180::_nrOfPossibleSolutions = 8;

void KatanaKinematics6M180::DK(coordinates &solution, encoders const &current_encoders) const
{
  using namespace KNI_MHF;
  // numering the angles starting by 0-5

  double factor;
  double R13, R23, R31, R32;

  angles current_angles(6);
  for (int z = 0; z < 6; ++z)
  {
    current_angles[z] = enc2rad(current_encoders[z], _parameters[z].angleOffset, _parameters[z].epc,
                                _parameters[z].encOffset, _parameters[z].rotDir);
  }

  // needs refactoring:
  current_angles[1] = current_angles[1] - M_PI / 2.0;
  current_angles[2] = current_angles[2] - M_PI;
  current_angles[3] = M_PI - current_angles[3];
  current_angles[4] = -current_angles[4];

  coordinates pose(6);

  angles cx(current_angles.size()), sx(current_angles.size());
  angles::iterator cx_iter, sx_iter;

  angles angle = current_angles;

  angle[2] = angle[1] + angle[2];
  angle[3] = angle[2] + angle[3];

  cx_iter = cx.begin();
  sx_iter = sx.begin();
  std::transform(angle.begin(), angle.end(), sx_iter, unary_precalc_sin< double >());
  std::transform(angle.begin(), angle.end(), cx_iter, unary_precalc_cos< double >());

  factor = (_length[0] * sx[1] + _length[1] * sx[2] + (_length[2] + _length[3]) * sx[3]);
  // x = px (compare homogenous transformation matrix)
  pose[0] = cx[0] * factor;

  // y = pz (compare homogenous transformation matrix)
  pose[1] = sx[0] * factor;

  // z = pz (compare homogenous transformation matrix)
  pose[2] = _length[0] * cx[1] + _length[1] * cx[2] + (_length[2] + _length[3]) * cx[3];

  // phi = atan2(R13/-R23) (compare homogenous transformatio nmatrix)
  R13 = cx[0] * sx[3];
  R23 = sx[0] * sx[3];
  pose[3] = atan2(R13, -R23);

  // theta = acos(R33) (compare homogenous transformation matrix)
  pose[4] = acos(cx[3]);

  // psi = atan2(R31/R32)  (compare homogenous transformation matrix)
  R31 = sx[3] * sx[4];
  R32 = sx[3] * cx[4];
  pose[5] = atan2(R31, R32);

  std::swap(solution, pose);
}

void KatanaKinematics6M180::init(metrics const &length, parameter_container const &parameters)
{
  assert((length.size() == 4) && "You have to provide the metrics for exactly 4 links");  // we have 4 links
  assert((parameters.size() == 6) &&
         "You have to provide exactly 5 motor parameters");  // 5 motors are used for IK calculations
  _setLength(length);
  _setParameters(parameters);
}

void KatanaKinematics6M180::IK_b1b2costh3_6M180(angles_calc &angle, const position &p) const
{
  using namespace KNI_MHF;

  double d5 = _length[2] + _length[3];

  angle.b1 = p.x * cos(angle.theta1) + p.y * sin(angle.theta1) - d5 * sin(angle.theta234);
  angle.b2 = p.z - d5 * cos(angle.theta234);
  angle.costh3 =
      -(pow2(angle.b1) + pow2(angle.b2) - pow2(_length[0]) - pow2(_length[1])) / (2.0 * _length[0] * _length[1]);
}

void KatanaKinematics6M180::thetacomp(angles_calc &angle, const position &p_m) const
{
  using namespace KNI_MHF;

  angle.theta2 = -M_PI / 2.0 - (atan0(angle.b1, angle.b2) +
                                atan0(_length[0] + _length[1] * cos(angle.theta3), _length[1] * sin(angle.theta3)));
  angle.theta4 = angle.theta234 - angle.theta2 - angle.theta3;

  if (!PositionTest6M180(angle, p_m))
  {
    angle.theta2 = angle.theta2 + M_PI;
    angle.theta4 = angle.theta234 - angle.theta2 - angle.theta3;
  }
}

bool KatanaKinematics6M180::PositionTest6M180(const angles_calc &a, const position &p) const
{
  using namespace KNI_MHF;
  double temp, xm2, ym2, zm2;

  temp =
      _length[0] * sin(a.theta2) + _length[1] * sin(a.theta2 + a.theta3) + (_length[2] + _length[3]) * sin(a.theta234);
  xm2 = cos(a.theta1) * temp;
  ym2 = sin(a.theta1) * temp;
  zm2 =
      _length[0] * cos(a.theta2) + _length[1] * cos(a.theta2 + a.theta3) + (_length[2] + _length[3]) * cos(a.theta234);

  if ((pow2(p.x - xm2) + pow2(p.y - ym2) + pow2(p.z - zm2)) >= _tolerance)
    return false;

  return true;
}

bool KatanaKinematics6M180::angledef(angles_calc &a) const
{
  using namespace KNI_MHF;

  // constants here. needs refactoring:
  a.theta2 = anglereduce(a.theta2 + M_PI / 2.0);
  a.theta3 = anglereduce(a.theta3 + M_PI);
  a.theta4 = anglereduce(M_PI - a.theta4);
  a.theta5 = anglereduce(a.theta5);

  if (a.theta1 > _parameters[0].angleStop)
  {
    a.theta1 = a.theta1 - 2.0 * M_PI;
  }
  if (a.theta2 > M_PI)
  {
    a.theta2 = a.theta2 - 2.0 * M_PI;
  }
  if (a.theta5 < _parameters[4].angleOffset)
  {
    a.theta5 = a.theta5 + 2.0 * M_PI;
  }

  return AnglePositionTest(a);
}

bool KatanaKinematics6M180::AnglePositionTest(const angles_calc &a) const
{
  if ((a.theta1 + 0.0087 < _parameters[0].angleOffset) || (a.theta1 > _parameters[0].angleStop))
    return false;

  if ((a.theta2 - 0.0087 > _parameters[1].angleOffset) || (a.theta2 < _parameters[1].angleStop))
    return false;

  if ((a.theta3 < _parameters[2].angleOffset) || (a.theta3 > _parameters[2].angleStop))
    return false;

  if ((a.theta4 < _parameters[3].angleOffset) || (a.theta4 > _parameters[3].angleStop))
    return false;

  if ((a.theta5 < _parameters[4].angleOffset) || (a.theta5 > _parameters[4].angleStop))
    return false;

  return true;
}

void KatanaKinematics6M180::IK(encoders::iterator solution, coordinates const &pose,
                               encoders const &current_encoders) const
{
  using namespace KNI_MHF;

  // pose: Winkel Deg->Rad
  // Alle 8 Loeungen werden in einem Array angle, welches aus 8 Structs besteht, gespeichert:
  // 0-3 fr theta1_1
  // 4-7 fr theta1_2

  // Declarations
  position p_m;
  angles_container angle(_nrOfPossibleSolutions);
  double coeff1, coeff2, theta234;
  double costh5, sinth5, theta5[2];
  double R11, R21, R31, R32;
  double phi, theta, psi;

  // Initialization
  p_m.x = pose[0];
  p_m.y = pose[1];
  p_m.z = pose[2];

  // calculate theta1_1 and theta1_2
  angle[0].theta1 = atan1(pose[0], pose[1]);
  if (angle[0].theta1 > M_PI)
  {
    angle[0].theta1 = angle[0].theta1 - M_PI;
    if (angle[0].theta1 > (179.91 / 180 * M_PI))
    {
      angle[0].theta1 = angle[0].theta1 - M_PI;
    }
  }
  angle[4].theta1 = angle[0].theta1 + M_PI;

  theta = pose[4];
  psi = pose[5];
  phi = atan1(p_m.x, p_m.y) + M_PI / 2.0;
  theta234 = pose[4];

  R11 = cos(phi) * cos(psi) - sin(phi) * cos(theta) * sin(psi);
  R21 = sin(phi) * cos(psi) + cos(phi) * cos(theta) * sin(psi);
  R31 = sin(theta) * sin(psi);
  R32 = sin(theta) * cos(psi);

  // calculate theta5
  if (theta234 == 0)
  {
    // std::cout << "Warning: Singularity theta234=0 !" << std::endl;
    for (int i = 0; i < 2; ++i)
    {
      coeff1 = -sin(angle[i * 4].theta1);
      coeff2 = -cos(angle[i * 4].theta1);
      costh5 = coeff1 * R11 - coeff2 * R21;
      sinth5 = coeff1 * R21 + coeff2 * R11;
      theta5[i] = -findFirstEqualAngle(costh5, sinth5, _tolerance);
    }
    for (int i = 0; i < _nrOfPossibleSolutions; ++i)
    {
      if (i < 4)
        angle[i].theta5 = theta5[0];
      else
        angle[i].theta5 = theta5[1];
    }
  }
  else if (theta234 == M_PI)
  {
    // std::cout << "Warning: Singularity theta234=PI !" << std::endl;
    for (int i = 0; i < 2; ++i)
    {
      coeff1 = -sin(angle[i * 4].theta1);
      coeff2 = cos(angle[i * 4].theta1);
      costh5 = coeff1 * R11 + coeff2 * R21;
      sinth5 = -coeff1 * R21 + coeff2 * R11;
      theta5[i] = -findFirstEqualAngle(costh5, sinth5, _tolerance);
    }
    for (int i = 0; i < _nrOfPossibleSolutions; ++i)
    {
      if (i < 4)
        angle[i].theta5 = theta5[0];
      else
        angle[i].theta5 = theta5[1];
    }
  }
  else
  {
    theta5[0] = -atan2(R31 / sin(theta234), R32 / sin(theta234));
    theta5[1] = -atan2(R31 / sin(-theta234), R32 / sin(-theta234));
    for (int i = 0; i < _nrOfPossibleSolutions; ++i)
    {
      if (i % 4 == 0 || i % 4 == 1)
        angle[i].theta5 = theta5[0];
      else
        angle[i].theta5 = theta5[1];
    }
  }

  //====THETA1_1==================
  //-------THETA234_1-------------
  angle[0].theta234 = pose[4];
  // angle[0].theta5 = pose[5];
  IK_b1b2costh3_6M180(angle[0], p_m);

  angle[1] = angle[0];
  angle[0].theta3 = acos(angle[0].costh3) - M_PI;
  thetacomp(angle[0], p_m);
  angle[1].theta3 = -acos(angle[1].costh3) + M_PI;
  thetacomp(angle[1], p_m);

  //-------THETA234_2-------------
  angle[2].theta1 = angle[0].theta1;
  angle[2].theta234 = -angle[0].theta234;
  // angle[2].theta5 = angle[0].theta5;
  IK_b1b2costh3_6M180(angle[2], p_m);

  angle[3] = angle[2];
  angle[2].theta3 = acos(angle[2].costh3) - M_PI;
  thetacomp(angle[2], p_m);
  angle[3].theta3 = -acos(angle[3].costh3) + M_PI;
  thetacomp(angle[3], p_m);

  //====THETA1_2==================
  //-------THETA234_1-------------
  angle[4].theta234 = pose[4];
  // angle[4].theta5 = pose[5];
  IK_b1b2costh3_6M180(angle[4], p_m);

  angle[5] = angle[4];
  angle[4].theta3 = acos(angle[4].costh3) - M_PI;
  thetacomp(angle[4], p_m);
  angle[5].theta3 = -acos(angle[5].costh3) + M_PI;
  thetacomp(angle[5], p_m);

  //-------THETA234_2-------------
  angle[6].theta1 = angle[4].theta1;
  angle[6].theta234 = -angle[4].theta234;
  // angle[6].theta5 = angle[4].theta5;
  IK_b1b2costh3_6M180(angle[6], p_m);

  angle[7] = angle[6];
  angle[6].theta3 = acos(angle[6].costh3) - M_PI;
  thetacomp(angle[6], p_m);
  angle[7].theta3 = -acos(angle[7].costh3) + M_PI;
  thetacomp(angle[7], p_m);

  for (std::vector< angles_calc >::iterator iter = angle.begin(); iter != angle.end(); /* do iter forward in body */)
  {
    if (pow2(iter->costh3) <= 1.0)
    {
      if (!angledef(*iter))
        iter = angle.erase(iter);
      else
        ++iter;
      continue;
    }
    iter = angle.erase(iter);
  }

  if (angle.size() == 0)
  {
    throw NoSolutionException();
  }

  std::vector< std::vector< int > > PossibleTargetsInEncoders;
  for (std::vector< angles_calc >::iterator i = angle.begin(); i != angle.end(); ++i)
  {
    std::vector< int > solution(5);

    solution[0] = rad2enc(i->theta1, _parameters[0].angleOffset, _parameters[0].epc, _parameters[0].encOffset,
                          _parameters[0].rotDir);
    solution[1] = rad2enc(i->theta2, _parameters[1].angleOffset, _parameters[1].epc, _parameters[1].encOffset,
                          _parameters[1].rotDir);
    solution[2] = rad2enc(i->theta3, _parameters[2].angleOffset, _parameters[2].epc, _parameters[2].encOffset,
                          _parameters[2].rotDir);
    solution[3] = rad2enc(i->theta4, _parameters[3].angleOffset, _parameters[3].epc, _parameters[3].encOffset,
                          _parameters[3].rotDir);
    solution[4] = rad2enc(i->theta5, _parameters[4].angleOffset, _parameters[4].epc, _parameters[4].encOffset,
                          _parameters[4].rotDir);

    PossibleTargetsInEncoders.push_back(solution);
  }

  std::vector< std::vector< int > >::const_iterator sol =
      KinematicsDefaultEncMinAlgorithm()(PossibleTargetsInEncoders.begin(), PossibleTargetsInEncoders.end(),
                                         current_encoders.begin(), current_encoders.end());

  assert(sol != PossibleTargetsInEncoders.end() && "All solutions are out of range");

  encoders::iterator gripper_encoder_iter = std::copy((*sol).begin(), (*sol).end(), solution);
  *gripper_encoder_iter = current_encoders[5];  // copy gripper-encoders from current
}

}  // NAMESPACE: KNI
