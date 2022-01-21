/***************************************************************************
 *   Copyright (C) 2006 by Tiziano Mueller   *
 *   tiziano.mueller@neuronics.ch   *
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

#include "KNI_InvKin/KatanaKinematics6M90T.h"
#include "common/MathHelperFunctions.h"
#include <algorithm>

namespace KNI
{
const double KatanaKinematics6M90T::_tolerance = 0.001;
const int KatanaKinematics6M90T::_nrOfPossibleSolutions = 8;

void KatanaKinematics6M90T::DK(coordinates& solution, encoders const& current_encoders) const
{
  using namespace KNI_MHF;
  // numering the angles starting by 0-5

  double x0, x1, x2, x3;
  double y0, y1, y2, y3;
  double z0, z1, z2, z3;

  angles current_angles(current_encoders.size());
  for (unsigned int z = 0; z < current_angles.size(); ++z)
  {
    current_angles[z] = enc2rad(current_encoders[z], _parameters[z].angleOffset, _parameters[z].epc,
                                _parameters[z].encOffset, _parameters[z].rotDir);
  }

  // needs refactoring:
  current_angles[1] = current_angles[1] - M_PI / 2.0;
  current_angles[2] = current_angles[2] - M_PI;
  current_angles[3] = M_PI - current_angles[3];
  current_angles[5] = -current_angles[5];

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

  // x
  x0 = cx[0] * sx[1];
  x1 = cx[0] * sx[2];
  x2 = cx[0] * sx[3];
  x3 = -cx[0] * cx[3] * cx[4] - sx[0] * sx[4];
  pose[0] = x0 * _length[0] + x1 * _length[1] + x2 * _length[2] + x3 * _length[3];

  // y
  y0 = sx[0] * sx[1];
  y1 = sx[0] * sx[2];
  y2 = sx[0] * sx[3];
  y3 = -sx[0] * cx[3] * cx[4] + cx[0] * sx[4];
  pose[1] = y0 * _length[0] + y1 * _length[1] + y2 * _length[2] + y3 * _length[3];

  // z
  z0 = cx[1];
  z1 = cx[2];
  z2 = cx[3];
  z3 = cx[4] * sx[3];
  pose[2] = z0 * _length[0] + z1 * _length[1] + z2 * _length[2] + z3 * _length[3];

  // theta
  pose[4] = acos(cx[4] * sx[3]);

  // phi & psi

  const double theta1 = angle[0];
  const double theta5 = angle[4];
  const double theta6 = angle[5];
  const double theta234 = angle[3];

  if (std::abs(pose[4]) < _tolerance || std::abs(pose[4] - M_PI) < _tolerance)
  {  // catch the case where theta=0, resp. theta=180
    // phi
    angles v1(2), v2(2);

    double R11 = -sin(theta1) * cos(theta5) * sin(theta6) +
                 cos(theta1) * (sin(theta234) * cos(theta6) + cos(theta234) * sin(theta5) * sin(theta6));
    double R21 = sin(theta1) * sin(theta234) * cos(theta6) +
                 sin(theta6) * (cos(theta1) * cos(theta5) + cos(theta234) * sin(theta1) * sin(theta5));

    v1[0] = acos(R11);
    v1[1] = -v1[0];
    v2[0] = asin(R21);
    v2[1] = M_PI - v2[0];

    pose[3] = anglereduce(findFirstEqualAngle(v1, v2));

    // psi
    pose[5] = 0;
  }
  else
  {
    // phi
    const double R13 = -cos(theta1) * cos(theta234) * cos(theta5) - sin(theta1) * sin(theta5);
    const double R23 = -sin(theta1) * cos(theta234) * cos(theta5) + cos(theta1) * sin(theta5);
    pose[3] = atan2(R13, -R23);

    // psi
    const double R31 = cos(theta234) * cos(theta6) - sin(theta234) * sin(theta5) * sin(theta6);
    const double R32 = -cos(theta234) * sin(theta6) - sin(theta234) * sin(theta5) * cos(theta6);
    pose[5] = atan2(R31, R32);
  }

  std::swap(solution, pose);
}

void KatanaKinematics6M90T::init(metrics const& length, parameter_container const& parameters)
{
  assert((length.size() == 4) && "You have to provide the metrics for exactly 4 links");  // we have 4 links
  assert((parameters.size() == 6) &&
         "You have to provide exactly 5 motor parameters");  // 5 motors are used for IK calculations
  _setLength(length);
  _setParameters(parameters);
}

void KatanaKinematics6M90T::IK_theta234theta5(angles_calc& angle, const position& p_gr) const
{
  using namespace KNI_MHF;

  if (p_gr.z == 0)
  {
    angle.theta234 = 0;
    angle.theta5 = angle.theta1 - atan1(-p_gr.x, -p_gr.y);
  }
  else
  {
    angle.theta234 =
        -acotan(((p_gr.x * p_gr.z * cos(angle.theta1)) -
                 sqrt((-pow2(p_gr.z)) * (-pow2(_length[3]) + pow2(p_gr.x) + pow2(p_gr.z)) * pow2(sin(angle.theta1)))) /
                pow2(p_gr.z));
    angle.theta5 = acos(p_gr.z / (_length[3] * sin(angle.theta234)));
  }

  bool griptest;
  griptest = GripperTest(p_gr, angle);
  if (!griptest)
  {
    angle.theta5 = -angle.theta5;
    griptest = GripperTest(p_gr, angle);
    if (!griptest)
    {
      angle.theta234 = -acotan(((p_gr.x * p_gr.z * cos(angle.theta1)) +
                                sqrt((-pow2(p_gr.z)) * (-pow2(_length[3]) + pow2(p_gr.x) + pow2(p_gr.z)) *
                                     pow2(sin(angle.theta1)))) /
                               pow2(p_gr.z));
      angle.theta5 = acos(p_gr.z / (_length[3] * sin(angle.theta234)));
      if (p_gr.z == 0)
      {
        angle.theta234 = -M_PI;
        angle.theta5 = atan1(p_gr.x, p_gr.y) - angle.theta1;
      }

      griptest = GripperTest(p_gr, angle);
      if (!griptest)
      {
        angle.theta5 = -angle.theta5;
      }
    }
  }
}

bool KatanaKinematics6M90T::GripperTest(const position& p_gr, const angles_calc& angle) const
{
  using namespace KNI_MHF;
  double xgr2, ygr2, zgr2;

  xgr2 = -_length[3] *
         (cos(angle.theta1) * cos(angle.theta234) * cos(angle.theta5) + sin(angle.theta1) * sin(angle.theta5));
  ygr2 = -_length[3] *
         (sin(angle.theta1) * cos(angle.theta234) * cos(angle.theta5) - cos(angle.theta1) * sin(angle.theta5));
  zgr2 = _length[3] * sin(angle.theta234) * cos(angle.theta5);

  if ((pow2(p_gr.x - xgr2) + pow2(p_gr.y - ygr2) + pow2(p_gr.z - zgr2)) >= _tolerance)
    return false;

  return true;
}

void KatanaKinematics6M90T::IK_b1b2costh3_6MS(angles_calc& angle, const position& p) const
{
  using namespace KNI_MHF;

  double xg, yg, zg;
  double d5 = _length[2] + _length[3];
  xg = p.x + (_length[3] * cos(angle.theta1) * sin(angle.theta234));
  yg = p.y + (_length[3] * sin(angle.theta1) * sin(angle.theta234));
  zg = p.z + (_length[3] * cos(angle.theta234));

  angle.b1 = xg * cos(angle.theta1) + yg * sin(angle.theta1) - d5 * sin(angle.theta234);
  angle.b2 = zg - d5 * cos(angle.theta234);
  angle.costh3 =
      -(pow2(angle.b1) + pow2(angle.b2) - pow2(_length[0]) - pow2(_length[1])) / (2.0 * _length[0] * _length[1]);
}

double KatanaKinematics6M90T::findFirstEqualAngle(const angles& v1, const angles& v2) const
{
  using namespace KNI_MHF;
  for (angles::const_iterator i = v1.begin(); i != v1.end(); ++i)
  {
    for (angles::const_iterator j = v2.begin(); j != v2.end(); ++j)
    {
      if (std::abs(anglereduce(*j) - anglereduce(*i)) < _tolerance)
        return *i;
    }
  }
  assert(!"precondition for findFirstEqualAngle failed -> no equal angles found");
  return 0;
}

void KatanaKinematics6M90T::thetacomp(angles_calc& angle, const position& p_m, const coordinates& pose) const
{
  using namespace KNI_MHF;

  const double theta1 = angle.theta1;
  double theta2 = 0;
  const double theta3 = angle.theta3;
  double theta4 = 0;
  const double theta5 = angle.theta5;
  double theta6 = 0;
  const double theta234 = angle.theta234;
  const double b1 = angle.b1;
  const double b2 = angle.b2;

  const double phi = pose[3];
  const double theta = pose[4];
  const double psi = pose[5];

  theta2 = -M_PI / 2.0 - (atan0(b1, b2) + atan0(_length[0] + _length[1] * cos(theta3), _length[1] * sin(theta3)));
  theta4 = theta234 - theta2 - theta3;

  if (!PositionTest6MS(theta1, theta2, theta3, theta234, p_m))
  {
    theta2 = theta2 + M_PI;
    theta4 = theta234 - theta2 - theta3;
  }

  const double R11 = cos(phi) * cos(psi) - sin(phi) * cos(theta) * sin(psi);
  const double R21 = sin(phi) * cos(psi) + cos(phi) * cos(theta) * sin(psi);

  std::vector< double > theta16c(2), theta16s(2);

  if (std::abs(theta234 + M_PI / 2) < _tolerance)
  {
    if (std::abs(theta5) < _tolerance)
    {
      theta16c[0] = acos(-R11);
      theta16c[1] = -theta16c[0];
      theta16s[0] = asin(-R21);
      theta16s[1] = M_PI - theta16s[0];

      theta6 = theta1 - findFirstEqualAngle(theta16c, theta16s);
    }
    else if (std::abs(theta5 - M_PI) < _tolerance)
    {
      theta16c[0] = acos(-R11);
      theta16c[1] = -theta16c[0];
      theta16s[0] = asin(-R21);
      theta16s[1] = M_PI - theta16s[0];

      theta6 = findFirstEqualAngle(theta16c, theta16s) - theta1;
    }
    else
    {
      assert(!"Special case \"|theta234+(1/2)*pi| = 0\" detected, but no solution found");
    }
  }
  else if (std::abs(theta234 + 3 * M_PI / 2) < _tolerance)
  {
    if (std::abs(theta5) < _tolerance)
    {
      theta16c[0] = acos(R11);
      theta16c[1] = -theta16c[0];
      theta16s[0] = asin(R21);
      theta16s[1] = M_PI - theta16s[0];

      theta6 = findFirstEqualAngle(theta16c, theta16s) - theta1;
    }
    else if (std::abs(theta5 - M_PI) < _tolerance)
    {
      theta16c[0] = acos(R11);
      theta16c[1] = -theta16c[0];
      theta16s[0] = asin(R21);
      theta16s[1] = M_PI - theta16s[0];

      theta6 = -theta1 - findFirstEqualAngle(theta16c, theta16s);
    }
    else
    {
      assert(!"Special case \"|theta234+(3/2)*pi| = 0\" detected, but no solution found");
    }
  }
  else
  {
    const double R31 = sin(theta) * sin(psi);
    const double R32 = sin(theta) * cos(psi);

    const double temp1 = cos(theta234);
    const double temp2 = -sin(theta234) * sin(theta5);

    const double c = (R31 * temp1 + R32 * temp2) / (pow2(temp1) + pow2(temp2));
    const double s = (R31 * temp2 - R32 * temp1) / (pow2(temp1) + pow2(temp2));

    theta16c[0] = acos(c);
    theta16c[1] = -theta16c[0];
    theta16s[0] = asin(s);
    theta16s[1] = M_PI - theta16s[0];

    theta6 = findFirstEqualAngle(theta16c, theta16s);
  }

  angle.theta2 = theta2;
  angle.theta4 = theta4;
  angle.theta6 = theta6;
}

bool KatanaKinematics6M90T::PositionTest6MS(const double& theta1, const double& theta2, const double& theta3,
                                            const double& theta234, const position& p) const
{
  using namespace KNI_MHF;
  double temp, xm2, ym2, zm2;

  temp = _length[0] * sin(theta2) + _length[1] * sin(theta2 + theta3) + _length[2] * sin(theta234);
  xm2 = cos(theta1) * temp;
  ym2 = sin(theta1) * temp;
  zm2 = _length[0] * cos(theta2) + _length[1] * cos(theta2 + theta3) + _length[2] * cos(theta234);

  if ((pow2(p.x - xm2) + pow2(p.y - ym2) + pow2(p.z - zm2)) >= _tolerance)
    return false;

  return true;
}

bool KatanaKinematics6M90T::angledef(angles_calc& a) const
{
  using namespace KNI_MHF;

  // constants here. needs refactoring:
  a.theta2 = anglereduce(a.theta2 + M_PI / 2.0);
  a.theta3 = anglereduce(a.theta3 + M_PI);
  a.theta4 = anglereduce(M_PI - a.theta4);
  a.theta5 = anglereduce(a.theta5);
  a.theta6 = -a.theta6;

  if (a.theta1 > _parameters[0].angleStop)
  {
    a.theta1 = a.theta1 - 2.0 * M_PI;
  }
  if (a.theta2 > M_PI)
  {
    a.theta2 = a.theta2 - 2.0 * M_PI;
  }
  if (a.theta6 < _parameters[5].angleOffset)
  {
    a.theta6 = a.theta6 + 2.0 * M_PI;
  }
  else if (a.theta6 > _parameters[5].angleStop)
  {
    a.theta6 = a.theta6 - 2.0 * M_PI;
  }
  if (a.theta5 < _parameters[4].angleOffset)
  {
    a.theta5 += 2.0 * M_PI;
  }

  return AnglePositionTest(a);
}

bool KatanaKinematics6M90T::AnglePositionTest(const angles_calc& a) const
{
  if ((a.theta1 + 0.0087 < _parameters[0].angleOffset) || (a.theta1 > _parameters[0].angleStop))
  {
    return false;
  }
  if ((a.theta2 - 0.0087 > _parameters[1].angleOffset) || (a.theta2 < _parameters[1].angleStop))
  {
    return false;
  }
  if ((a.theta3 < _parameters[2].angleOffset) || (a.theta3 > _parameters[2].angleStop))
  {
    return false;
  }

  if ((a.theta4 < _parameters[3].angleOffset) || (a.theta4 > _parameters[3].angleStop))
  {
    return false;
  }

  if ((a.theta5 < _parameters[4].angleOffset) || (a.theta5 > _parameters[4].angleStop))
  {
    return false;
  }
  if ((a.theta6 < _parameters[5].angleOffset) || (a.theta6 > _parameters[5].angleStop))
  {
    return false;
  }

  return true;
}

void KatanaKinematics6M90T::IK(encoders::iterator solution, coordinates const& pose,
                               encoders const& current_encoders) const
{
  using namespace KNI_MHF;

  // pose: Winkel Deg->Rad
  // Alle 8 Loeungen werden in einem Array angle, welches aus 8 Structs besteht, gespeichert:
  // 0-3 fr theta1_1
  // 4-7 fr theta1_2

  // Declaration
  position p_gr;
  position p_m;
  angles_container angle(_nrOfPossibleSolutions);

  // calculation of the gripper vector
  p_gr.x = _length[3] * sin(pose[4]) * sin(pose[3]);
  p_gr.y = -_length[3] * sin(pose[4]) * cos(pose[3]);
  p_gr.z = _length[3] * cos(pose[4]);

  p_m.x = pose[0] - p_gr.x;
  p_m.y = pose[1] - p_gr.y;
  p_m.z = pose[2] - p_gr.z;

  // calculate theta1_1 and theta1_2
  angle[0].theta1 = atan1(p_m.x, p_m.y);
  angle[4].theta1 = angle[0].theta1 + M_PI;

  // check the borders according to the settings
  if (angle[0].theta1 > _parameters[0].angleStop)
    angle[0].theta1 = angle[0].theta1 - 2.0 * M_PI;

  if (angle[0].theta1 < _parameters[0].angleOffset)
    angle[0].theta1 = angle[0].theta1 + 2.0 * M_PI;

  if (angle[4].theta1 > _parameters[0].angleStop)
    angle[4].theta1 = angle[4].theta1 - 2.0 * M_PI;

  if (angle[4].theta1 < _parameters[0].angleOffset)
    angle[4].theta1 = angle[4].theta1 + 2.0 * M_PI;

  //====THETA1_1==================
  //-------THETA234_1-------------
  IK_theta234theta5(angle[0], p_gr);
  IK_b1b2costh3_6MS(angle[0], p_m);

  angle[1] = angle[0];
  angle[0].theta3 = acos(angle[0].costh3) - M_PI;
  thetacomp(angle[0], p_m, pose);
  angle[1].theta3 = -acos(angle[1].costh3) + M_PI;
  thetacomp(angle[1], p_m, pose);

  //-------THETA234_2-------------
  angle[2].theta1 = angle[0].theta1;
  angle[2].theta234 = angle[0].theta234 - M_PI;
  angle[2].theta5 = M_PI - angle[0].theta5;

  IK_b1b2costh3_6MS(angle[2], p_m);
  angle[3] = angle[2];
  angle[2].theta3 = acos(angle[2].costh3) - M_PI;
  thetacomp(angle[2], p_m, pose);
  angle[3].theta3 = -acos(angle[3].costh3) + M_PI;
  thetacomp(angle[3], p_m, pose);

  //====THETA1_2==================
  //-------THETA234_1-------------
  IK_theta234theta5(angle[4], p_gr);
  IK_b1b2costh3_6MS(angle[4], p_m);

  angle[5] = angle[4];
  angle[4].theta3 = acos(angle[4].costh3) - M_PI;
  thetacomp(angle[4], p_m, pose);
  angle[5].theta3 = -acos(angle[5].costh3) + M_PI;
  thetacomp(angle[5], p_m, pose);

  //-------THETA234_2-------------
  angle[6].theta1 = angle[4].theta1;
  angle[6].theta234 = angle[4].theta234 - M_PI;
  angle[6].theta5 = M_PI - angle[4].theta5;
  IK_b1b2costh3_6MS(angle[6], p_m);
  angle[7] = angle[6];
  angle[6].theta3 = acos(angle[6].costh3) - M_PI;
  thetacomp(angle[6], p_m, pose);
  angle[7].theta3 = -acos(angle[7].costh3) + M_PI;
  thetacomp(angle[7], p_m, pose);

  for (::std::vector< angles_calc >::iterator iter = angle.begin(); iter != angle.end(); /* do iter forward in body */)
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

  ::std::vector<::std::vector< int > > PossibleTargetsInEncoders;
  for (::std::vector< angles_calc >::iterator i = angle.begin(); i != angle.end(); ++i)
  {
    ::std::vector< int > solution(6);

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
    solution[5] = rad2enc(i->theta6, _parameters[5].angleOffset, _parameters[5].epc, _parameters[5].encOffset,
                          _parameters[5].rotDir);
    PossibleTargetsInEncoders.push_back(solution);
  }

  ::std::vector<::std::vector< int > >::const_iterator sol =
      KinematicsDefaultEncMinAlgorithm()(PossibleTargetsInEncoders.begin(), PossibleTargetsInEncoders.end(),
                                         current_encoders.begin(), current_encoders.end());

  assert(sol != PossibleTargetsInEncoders.end() && "All solutions are out of range");

  encoders::iterator gripper_encoder_iter = std::copy((*sol).begin(), (*sol).end(), solution);
}

}  // NAMESPACE: KNI
