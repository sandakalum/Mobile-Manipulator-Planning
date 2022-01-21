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
#ifndef KNIKATANAKINEMATICS6M90G_H
#define KNIKATANAKINEMATICS6M90G_H

#include "common/dllexport.h"

#include "KNI_InvKin/KatanaKinematics.h"
#include "KNI_InvKin/KatanaKinematicsDecisionAlgorithms.h"

#include <vector>

namespace KNI
{
/**
  @author Tiziano Mueller <tiziano.mueller@neuronics.ch>
  @author Christoph Voser <christoph.voser@neuronics.ch>
*/

class DLLDIR_IK KatanaKinematics6M90G : public KatanaKinematics
{
public:
  void init(metrics const &length, parameter_container const &parameters);

  // strong guarantee provided here:
  void DK(coordinates &solution, encoders const &current_encoders) const;
  void IK(encoders::iterator solution, coordinates const &pose, encoders const &cur_angles) const;

private:
  struct position
  {
    double x;
    double y;
    double z;
  };

  struct angles_calc
  {
    double theta1;
    double theta2;
    double theta3;
    double theta4;
    double theta5;
    double theta234;
    double b1;
    double b2;
    double costh3;
  };

  typedef std::vector< angles_calc > angles_container;

  metrics _length;
  parameter_container _parameters;

  static const double _tolerance;  // initialized in .cpp
  static const int _nrOfPossibleSolutions;

  void _setLength(metrics const &length)
  {
    _length = length;
  }
  void _setParameters(parameter_container const &parameters)
  {
    _parameters = parameters;
  }

  void IK_theta234theta5(angles_calc &angle, const position &p_gr) const;
  void IK_b1b2costh3_6MS(angles_calc &a, const position &p) const;

  void thetacomp(angles_calc &a, const position &p_m) const;

  bool angledef(angles_calc &a) const;

  bool GripperTest(const position &p_gr, const angles_calc &angle) const;
  bool AnglePositionTest(const angles_calc &a) const;
  bool PositionTest6MS(const angles_calc &a, const position &p) const;
};
}

#endif
