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

/******************************************************************************************************************/
#ifndef _IKBASE_H_
#define _IKBASE_H_
/******************************************************************************************************************/

#include "common/exception.h"
#include "common/dllexport.h"

#include "KNI/kmlExt.h"
#include "KNI/kmlCommon.h"

#include "KNI_InvKin/KatanaKinematics.h"
#include "KNI_InvKin/KatanaKinematics6M90G.h"
#include "KNI_InvKin/KatanaKinematics6M90T.h"
#include "KNI_InvKin/KatanaKinematics6M180.h"
#include "KNI_InvKin/KatanaKinematics5M180.h"

#include <vector>
#include <memory>
#include <cmath>

/******************************************************************************************************************/

#ifndef TM_ENDLESS
#define TM_ENDLESS -1  //!< timeout symbol for 'endless' waiting
#endif

class DLLDIR_IK CikBase : public CKatana
{
private:
  std::auto_ptr< KNI::KatanaKinematics > _kinematicsImpl;
  bool _kinematicsIsInitialized;
  void _initKinematics();

public:
  CikBase() : _kinematicsIsInitialized(false){};
  ~CikBase();

  /// Returns the version number of the kinematics used.
  /// @param version		vector to write in version (major, minor, revision)
  /// \note integrated analytical kinematics returns version 0.1.0
  /// \note kinematics library returns versions >= 1.0.0
  void getKinematicsVersion(std::vector< int >& version);

  /// Set the offset from the flange to the desired tcp
  /// @param xoff		offset in x direction of flange coordinate system in m
  /// @param yoff		offset in y direction of flange coordinate system in m
  /// @param zoff		offset in z direction of flange coordinate system in m
  /// @param psioff	angle offset around x-axis of flange coordinate system in rad
  void setTcpOffset(double xoff, double yoff, double zoff, double psioff);

  /// Returns the current position of the robot in cartesian coordinates.
  /// \note This method is deprecated, please use getCoordinates(...) instead
  void DKApos(double* position);

  /// Returns the current position of the robot in cartesian coordinates.
  /// @param refreshEncoders With this parameter you can determine if the method
  /// reads the actual encoders from the robot or if it will use the cached ones
  /// \note This function returns a tuple in python
  void getCoordinates(double& x, double& y, double& z, double& phi, double& theta, double& psi,
                      bool refreshEncoders = true);

  /// Returns the position of the robot corresponting to the given encoders in cartesian coordinates.
  void getCoordinatesFromEncoders(std::vector< double >& pose, const std::vector< int >& encs);

  /// Calculates a set of encoders for the given coordinates.
  /// This method reads the current encoders from the robot and
  /// involves therefore also communication to the robot
  void IKCalculate(double X, double Y, double Z, double Al, double Be, double Ga,
                   std::vector< int >::iterator solution_iter);

  /// Calculates a set of encoders for the given coordinates.
  /// For this method you have to pass an actualPosition too.
  /// No communication with the robot will be done here.
  void IKCalculate(double X, double Y, double Z, double Al, double Be, double Ga,
                   std::vector< int >::iterator solution_iter, const std::vector< int >& actualPosition);

  /// Moves to robot to given cartesian coordinates and euler-angles.
  /// \note This method is deprecated, please use moveRobotTo(...) instead
  void IKGoto(double X, double Y, double Z, double Al, double Be, double Ga, bool wait = false, int tolerance = 100,
              long timeout = TM_ENDLESS);

  /// Moves to robot to given cartesian coordinates and euler-angles.
  /// \note Instead of a given tolerance, a default tolerance is being used
  void moveRobotTo(double x, double y, double z, double phi, double theta, double psi, bool waitUntilReached = false,
                   int waitTimeout = TM_ENDLESS);

  /// This method does the same as the one above and is mainly provided for convenience
  /// \note You can call this function in python using tuples:
  ///       Example: katana.moveRobotTo( (x,y,z,phi,theta,psi) )
  /// \note If the size of the container is smaller than 6, it will throw an exception
  void moveRobotTo(std::vector< double > coordinates, bool waitUntilReached = false, int waitTimeout = TM_ENDLESS);
};

/******************************************************************************************************************/
#endif  //_IKBASE_H_
/******************************************************************************************************************/
