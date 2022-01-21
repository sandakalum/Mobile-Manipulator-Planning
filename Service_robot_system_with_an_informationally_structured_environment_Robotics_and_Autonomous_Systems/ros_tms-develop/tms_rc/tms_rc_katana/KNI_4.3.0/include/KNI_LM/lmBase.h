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

/********************************************************************************/
#ifndef _LMBASE_H_
#define _LMBASE_H_
/********************************************************************************/
#include "common/dllexport.h"
#include "common/Timer.h"
#include "KNI_InvKin/ikBase.h"
#include "common/exception.h"
#include <vector>
#include <cmath>

#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/triangular.hpp"
#include "boost/numeric/ublas/lu.hpp"
#include "boost/numeric/ublas/io.hpp"

/********************************************************************************/

//---------------------------------------------------------------------------//

///
/// @addtogroup exceptions
/// @{
///

/// Joint speed too high.
/// \note error_number = -70
class JointSpeedException : public Exception
{
public:
  JointSpeedException() throw() : Exception("Joint speed too high", -70)
  {
  }
};

/// Wait parameter set to false.
/// \note error_number = -71
class WaitParameterException : public Exception
{
public:
  WaitParameterException() throw() : Exception("Wait parameter set to false", -71)
  {
  }
};

//---------------------------------------------------------------------------//

/*!	\brief	Linear movement Class
 *
 *	This class allows to do linear movements with the Katana robot.
 */
class DLLDIR_LM CLMBase : public CikBase
{
private:
  double _maximumVelocity;
  bool _activatePositionController;

  /**
   * Calculates time needed for movement over a distance.
   *
   * @author Jonas Haller
   * @param distance	distance of the movement in mm
   * @param acc	acceleration at the beginning in mm/s^2
   * @param dec	deceleration at the end in mm/s^2
   * @param vmax	maximum velocity of the movement in mm/s
   * @return time needed for the movement in s
   */
  double totalTime(double distance, double acc, double dec, double vmax);

  /**
   * Calculates the relative position reached after the relative time given.
   *
   * @author Jonas Haller
   * @param reltime	relative time (fraction of totaltime)
   * @param distance	distance of the movement in mm
   * @param acc	acceleration at the beginning in mm/s^2
   * @param dec	deceleration at the end in mm/s^2
   * @param vmax	maximum velocity of the movement in mm/s
   * @return relative distance (fraction of distance)
   */
  double relPosition(double reltime, double distance, double acc, double dec, double vmax);

  /**
   * Calculates the spline coefficient and stores them in arr_p1 - arr_p4.
   *
   * Boundary conditions are that f_1'=0 and f_n'=0 (zero velocity at beginning
   * and end of the movement) and f_i''=P_(i+1)''.
   *
   * @author Jonas Haller
   * @param steps	number of splines to calculate
   * @param timearray	times of the points (length = steps + 1)
   * @param encoderarray	encoder values of the points (length = steps + 1)
   * @param arr_p1	to return parameters 1 (length = steps)
   * @param arr_p2	to return parameters 2 (length = steps)
   * @param arr_p3	to return parameters 3 (length = steps)
   * @param arr_p4	to return parameters 4 (length = steps)
   * @return void
   */
  void splineCoefficients(int steps, double *timearray, double *encoderarray, double *arr_p1, double *arr_p2,
                          double *arr_p3, double *arr_p4);

  /**
   * Checks if the joint speeds are below speed limit.
   *
   * Maximum joint speed is 180enc / 10ms.
   *
   * @author Jonas Haller
   * @param lastsolution	encoder values of last point
   * @param solution	encoder values of current point
   * @param time	time difference between the points in s
   * @return true if joint speeds ok, false if joint speed too high
   */
  bool checkJointSpeed(std::vector< int > lastsolution, std::vector< int > solution, double time);

  /**
   * Calculates speed from distance, acceleration and time for the movement.
   *
   * @author Jonas Haller
   * @param distance	absolute (positive) distance of the movement in encoder
   * @param acceleration	acceleration and deceleration in enc / s^2
   * @param time	time that can be used for the movement in s
   * @return speed in enc / s to finish the movement on time
   */
  int getSpeed(int distance, int acceleration, int time);

public:
  CLMBase() : _maximumVelocity(20), _activatePositionController(true)
  {
  }

  /// @param wait has to be true with new implementation of movLM2P
  void movLM(double X, double Y, double Z, double Al, double Be, double Ga, bool exactflag, double vmax,
             bool wait = true, int tolerance = 100, long timeout = TM_ENDLESS);

  /**
   * Move linear from point to point using multiple splines.
   *
   * @author Jonas Haller
   * @param X1, Y1, Z1, Ph1, Th1, Ps1
   * 		X, Y, Z, Phi, Theta, Psi of actual position
   * @param X2, Y2, Z2, Ph2, Th2, Ps2
   * 		X, Y, Z, Phi, Theta, Psi of target position
   * @param exactflag	activate the position controller after the movement
   * @param vmax	maximum velocity of the movement in mm/s
   * @param wait	wait for end of movement
   * @param tolerance	tolerance for all motor encoders
   * @param timeout	timeout for linear movement in ms
   * @throws NoSolutionException	if no solution found for IK
   * @throws JointSpeedException	if joint speed too high
   * @throws WaitParameterException	if wait set to false
   * @return void
   */
  void movLM2P(double X1, double Y1, double Z1, double Al1, double Be1, double Ga1, double X2, double Y2, double Z2,
               double Al2, double Be2, double Ga2, bool exactflag, double vmax, bool wait = true, int tolerance = 100,
               long timeout = TM_ENDLESS);

  /**
   * Move point to point using splines.
   *
   * @author Jonas Haller
   * @param X1, Y1, Z1, Ph1, Th1, Ps1
   * 		X, Y, Z, Phi, Theta, Psi of actual position
   * @param X2, Y2, Z2, Ph2, Th2, Ps2
   * 		X, Y, Z, Phi, Theta, Psi of target position
   * @param exactflag	activate the position controller after the movement
   * @param vmax	maximum velocity for motors
   * @param wait	wait for end of movement
   * @param tolerance	tolerance for all motor encoders
   * @param timeout	timeout for movement in ms
   * @throws NoSolutionException	if no solution found for IK
   * @throws JointSpeedException	if joint speed too high
   * @throws WaitParameterException	if wait set to false
   * @return void
   */
  void movP2P(double X1, double Y1, double Z1, double Ph1, double Th1, double Ps1, double X2, double Y2, double Z2,
              double Ph2, double Th2, double Ps2, bool exactflag, double vmax, bool wait = true,
              long timeout = TM_ENDLESS);

  void setMaximumLinearVelocity(double maximumVelocity);
  double getMaximumLinearVelocity() const;

  ///
  /// Re-Activate the position controller after the linear movement.
  /// \note This can result in a small movement after the movement
  void setActivatePositionController(bool activate);

  ///
  /// Check if the position controller will be activated after the linear movement
  bool getActivatePositionController();

  /// @param waitUntilReached has to be true with new implementation of movLM2P
  void moveRobotLinearTo(double x, double y, double z, double phi, double theta, double psi,
                         bool waitUntilReached = true, int waitTimeout = TM_ENDLESS);

  /// This method does the same as the one above and is mainly provided for convenience
  /// \note You can call this function in python using tuples:
  ///       Example: katana.moveRobotLinearTo( (x,y,z,phi,theta,psi) )
  /// \note If the size of the container is smaller than 6, it will throw an exception!
  void moveRobotLinearTo(std::vector< double > coordinates, bool waitUntilReached = true, int waitTimeout = TM_ENDLESS);

  /// Moves to robot to given cartesian coordinates and euler-angles.
  /// \note Instead of a given tolerance, a default tolerance is being used
  void moveRobotTo(double x, double y, double z, double phi, double theta, double psi, bool waitUntilReached = true,
                   int waitTimeout = TM_ENDLESS);

  /// This method does the same as the one above and is mainly provided for convenience
  /// \note You can call this function in python using tuples:
  ///       Example: katana.moveRobotTo( (x,y,z,phi,theta,psi) )
  /// \note If the size of the container is smaller than 6, it will throw an exception
  void moveRobotTo(std::vector< double > coordinates, bool waitUntilReached = true, int waitTimeout = TM_ENDLESS);
};
/********************************************************************************/
#endif  //_IKBASE_H_
/********************************************************************************/
