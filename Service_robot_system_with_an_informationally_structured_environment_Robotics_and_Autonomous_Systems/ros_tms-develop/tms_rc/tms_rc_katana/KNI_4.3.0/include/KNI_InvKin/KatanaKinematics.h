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
#ifndef KATANAKINEMATICS_H
#define KATANAKINEMATICS_H

#include "common/dllexport.h"
#include "common/exception.h"

#include <vector>

namespace KNI
{
///
/// @addtogroup exceptions
/// @{
///

/// No solution found for the given cartesian coordinates.
/// \note error_number=-60
class NoSolutionException : public Exception
{
public:
  NoSolutionException() throw() : Exception("No solution found", -60)
  {
  }
};

///
/// @}
///

/// To pass different parameters for the kinematic implementations.
/// These parameters are used for "reducing" different solutions to
/// valid angles and to to check angles against given limits (angleOffset, angleStop)
struct DLLDIR_IK KinematicParameters
{
  double angleOffset;
  double angleStop;
  int epc;
  int encOffset;
  int rotDir;
};

///
/// The base class for all kinematic implementations.
class DLLDIR_IK KatanaKinematics
{
public:
  virtual ~KatanaKinematics()
  {
  }

  typedef std::vector< KinematicParameters > parameter_container;

  ///
  /// Being used to store angles (in radian).
  typedef std::vector< double > angles;
  ///
  /// To store coordinates.
  typedef std::vector< double > coordinates;
  ///
  /// To store metrics, 'aka' the length's of the different segments of the robot.
  typedef std::vector< double > metrics;
  ///
  /// To store encoders.
  typedef std::vector< int > encoders;

  /// Initialize the parameters for the calculations.
  /// This is needed to validate the calculated angles and to choose an appropriate solution
  /// You have to provide 5 or 6 length's and parameters, depending on you robot type
  virtual void init(metrics const& length, parameter_container const& parameters) = 0;

  /// Direct Kinematic.
  /// Calculates the actual position in cartesian coordinates using the given encoders
  /// @param solution This is where the algorithm will store the solution to (in cartesian coordinates)
  /// @param current_encoders The encoder values which are being used for the calculation
  /// \note strong guarantee provided
  virtual void DK(coordinates& solution, encoders const& current_encoders) const = 0;

  /// Inverse Kinematic.
  /// Calculates one set of encoders (=one solution) for the given cartesian coordinates.
  /// You also have to provide the current encoders to allow the algorithm to choose between different
  /// valid solutions.
  /// @param solution This is where the algorithm will store the solution to (in encoders)
  /// @param pose The target position in cartesian coordinates plus the euler angles for the direction of the gripper
  /// @param cur_angles The current angles (in encoders) of the robot
  /// \note strong guarantee provided
  virtual void IK(encoders::iterator solution, coordinates const& pose, encoders const& cur_angles) const = 0;
};
}

#endif
