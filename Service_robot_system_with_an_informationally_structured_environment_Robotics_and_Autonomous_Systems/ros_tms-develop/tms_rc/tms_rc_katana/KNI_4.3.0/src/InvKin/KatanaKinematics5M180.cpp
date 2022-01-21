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

#include "KNI_InvKin/KatanaKinematics5M180.h"
#include "common/MathHelperFunctions.h"

namespace KNI
{
const double KatanaKinematics5M180::_tolerance = 0.001;
const int KatanaKinematics5M180::_nrOfPossibleSolutions = 1;

void KatanaKinematics5M180::DK(coordinates& solution, encoders const& current_encoders) const
{
  using namespace KNI_MHF;
  // numbering the angles by 0-4
  double a, b, c, alpha1, alpha2;
  coordinates pose(6, 0);

  // size has to be 5
  angles current_angles(5);
  for (unsigned int z = 0; z < current_encoders.size(); ++z)
  {
    current_angles[z] = enc2rad(current_encoders[z], _parameters[z].angleOffset, _parameters[z].epc,
                                _parameters[z].encOffset, _parameters[z].rotDir);
  }

  a = _length[1] + _length[2];
  b = _length[0];
  c = sqrt(pow2(a) + pow2(b) - 2 * a * b * cos(current_angles[2]));  // law of cosinef
  alpha1 = asin(a * sin(current_angles[2]) / c);                     // law of sine
  alpha2 = current_angles[1] - alpha1;

  pose[0] = c * cos(alpha2) * cos(current_angles[0]);
  pose[1] = c * cos(alpha2) * sin(current_angles[0]);
  pose[2] = c * sin(alpha2);

  std::swap(solution, pose);
}

void KatanaKinematics5M180::init(metrics const& length, parameter_container const& parameters)
{
  assert((length.size() == 3) && "You have to provide the metrics for exactly 3 links");  // we have 3 links
  assert((parameters.size() == 5) &&
         "You have to provide exactly 5 motor parameters");  // 5 motors are used for IK calculations
  _setLength(length);
  _setParameters(parameters);
}

void KatanaKinematics5M180::IK(encoders::iterator solution, coordinates const& pose,
                               encoders const& current_encoders) const
{
  using namespace KNI_MHF;

  // pose: Winkel Deg->Rad
  // Nur eine Lösung

  // Declarations
  position p_m;
  angles_container angle(_nrOfPossibleSolutions);

  double dist, a, b;
  double alpha1Px, alpha2Px;

  // Initialization
  p_m.x = pose[0];
  p_m.y = pose[1];
  p_m.z = pose[2];

  dist = sqrt(pow2(p_m.x) + pow2(p_m.y) + pow2(p_m.z));
  alpha2Px = asin(p_m.z / dist);

  angle[0].theta1 = atan1(p_m.x, p_m.y);
  if (angle[0].theta1 > (2 * M_PI + _parameters[0].angleOffset))
    angle[0].theta1 -= 2 * M_PI;

  a = _length[1] + _length[2];
  b = _length[0];
  angle[0].theta3 = acos((pow2(a) + pow2(b) - pow2(dist)) / (2 * a * b));  // law of cosinef
  if (angle[0].theta3 > (2 * M_PI + _parameters[2].angleOffset))
    angle[0].theta3 -= 2 * M_PI;

  alpha1Px = asin(a * sin(angle[0].theta3) / dist);
  angle[0].theta2 = alpha1Px + alpha2Px;
  if (angle[0].theta2 > (2 * M_PI + _parameters[1].angleOffset))
    angle[0].theta2 -= 2 * M_PI;

  std::vector< int > temp_solution(5);

  temp_solution[0] = rad2enc(angle[0].theta1, _parameters[0].angleOffset, _parameters[0].epc, _parameters[0].encOffset,
                             _parameters[0].rotDir);
  temp_solution[1] = rad2enc(angle[0].theta2, _parameters[1].angleOffset, _parameters[1].epc, _parameters[1].encOffset,
                             _parameters[1].rotDir);

  temp_solution[2] = rad2enc(angle[0].theta3, _parameters[2].angleOffset, _parameters[2].epc, _parameters[2].encOffset,
                             _parameters[2].rotDir);
  temp_solution[3] = current_encoders[3];  // copy encoders
  temp_solution[4] = current_encoders[4];  // copy encoders

  std::copy(temp_solution.begin(), temp_solution.end(), solution);
}

}  // NAMESPACE: KNI
