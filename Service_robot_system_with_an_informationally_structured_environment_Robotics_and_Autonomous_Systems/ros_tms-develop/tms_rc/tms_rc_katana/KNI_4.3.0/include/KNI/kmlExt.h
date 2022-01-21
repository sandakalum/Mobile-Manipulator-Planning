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
#ifndef _KMLEXT_H_
#define _KMLEXT_H_
/******************************************************************************************************************/
#include "common/dllexport.h"
#include "common/exception.h"

#include "KNI/kmlBase.h"
#include "KNI/kmlMotBase.h"

#include <vector>

/******************************************************************************************************************/

///
/// @addtogroup exceptions
/// @{
///

///
/// Accessing the given configuration file failed (may be: access denied or wrong path)
/// \note error_number=-40
class ConfigFileOpenException : public Exception
{
public:
  ConfigFileOpenException(const std::string& port) throw()
    : Exception("Cannot open configuration file '" + port + "'", -40)
  {
  }
};

///
/// @}
///

namespace KNI
{
class kmlFactory;  /// Forward declaration
}

/*!	\brief	Extended Katana class with additional functions
 *
 *	This class uses the 'CKatBase* base' object to refer to a Katana robot.
 */

class DLLDIR CKatana
{
protected:
  //-------------------------------------//
  CKatBase* base;  //!< base katana

  bool _gripperIsPresent;
  int _gripperOpenEncoders;
  int _gripperCloseEncoders;
  //!The type of KatanaXXX (300 or 400)
  int mKatanaType;
  //!The kinematics implementation: 0 for Analytical, 1 for RobAnaGuess
  int mKinematics;

  /*!	\brief	Sets the tolerance range in encoder units for the robots movements.
  */
  void setTolerance(long idx, int enc_tolerance);

public:
  //-------------------------------------//
  CKatBase* GetBase()
  {
    return base;
  }  //!< Returns pointer to 'CKatBase*'

  /*!	\brief Constructor

   */
  CKatana()
  {
    base = new CKatBase;
  }
  /*!	\brief Destructor
   */
  ~CKatana()
  {
    delete base;
  }
  //------------------------------------------------------------------------------//
  /*! \brief Create routine
   */
  void create(const char* configurationFile, CCplBase* protocol);
  void create(KNI::kmlFactory* infos, CCplBase* protocol);

  /*! \brief Create routine
   */
  void create(TKatGNL& gnl,       //!< katana initial attributes
              TKatMOT& mot,       //!< motor initial attributes
              TKatSCT& sct,       //!< sensor controller initial attributes
              TKatEFF& eff,       //!< end effector initial attributes
              CCplBase* protocol  //!< protocol to be used
              );
  //------------------------------------------------------------------------------//

  /* \brief calibrates the robot
   */
  void calibrate();

  void calibrate(long idx,     //!< motor index
                 TMotCLB clb,  //!< calibration struct for one motor
                 TMotSCP scp,  //!< static controller parameters
                 TMotDYL dyl   //!< dynamic controller parameters
                 );

  //------------------------------------------------------------------------------//

  void searchMechStop(long idx,        //!< motor index
                      TSearchDir dir,  //!< search direction
                      TMotSCP scp,     //!< static controller parameters
                      TMotDYL dyl      //!< dynamic controller parameters
                      );

  //------------------------------------------------------------------------------//
  /*!	\brief	Increments the motor specified by an index postion in encoders.
   */
  void inc(long idx, int dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Decrements the motor specified by an index postion in encoders.
   */
  void dec(long idx, int dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Moves the motor specified by an index to a given target position
   *			in encoders.
   */
  void mov(long idx, int tar, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);

  //------------------------------------------------------------------------------//
  /*!	\brief	Increments the motor specified by an index postion in degree units.
   */
  void incDegrees(long idx, double dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Decrements the motor specified by an index postion in degree units.
   */
  void decDegrees(long idx, double dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Moves the motor specified by an index to a given target position
   *			in degree units.
   */
  void movDegrees(long idx, double tar, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);

  //------------------------------------------------------------------------------//
  // public just for dubbuging purposes
  /*!	\brief	Check if the absolute position in degrees is out of range.
   */

  bool checkENLD(long idx, double degrees);

  //------------------------------------------------------------------------------//

  /// Tell the robot about the presence of a gripper.
  /// @param openEncoders Which encoders should be used as target positions for opening the gripper
  /// @param closeEncoders Dito for closing the gripper
  void setGripperParameters(bool isPresent, int openEncoders, int closeEncoders);
  /// Get the gripper parameters
  void getGripperParameters(bool& isPresent, int& openEncoders, int& closeEncoders);
  //------------------------------------------------------------------------------//

  /*!\brief crash limits enable
  */
  void enableCrashLimits();
  /*!\brief crash limits disable
  */
  void disableCrashLimits();
  /*!\brief unblock robot after a crash
  */
  void unBlock();
  /*!\brief flush move buffers
  */
  void flushMoveBuffers();
  /*!\brief unblock robot after a crash
  */
  void setCrashLimit(long idx, int limit);
  /*!\brief set collision position limits
   */
  void setPositionCollisionLimit(long idx, int limit);
  /*!\brief set collision speed limits
   */
  void setSpeedCollisionLimit(long idx, int limit);
  /*!\brief Set the force limit for the current controller
  @param axis axis number 1-6. If 0, send limit to all axes
   */
  void setForceLimit(int axis, int limit);

  //------------------------------------------------------------------------------//

  short getNumberOfMotors() const;
  int getMotorEncoders(short number, bool refreshEncoders = true) const;

  /// Write the cached encoders into the container. Set refreshEncoders=true if the KNI should fetch them from the
  /// robot.
  /// If m=distance(start, end) is smaller than the number of motors, only the first m motors will be written to the
  /// container,
  /// the function will not throw an exception because of this. The return value will point to one element after the
  /// last one.
  std::vector< int >::iterator getRobotEncoders(std::vector< int >::iterator start,
                                                std::vector< int >::const_iterator end,
                                                bool refreshEncoders = true) const;

  /// Get the current robot encoders as a vector-container.
  /// This method is mainly provided for convenience. It is easier than the other getRobotEncoders method
  /// but probably not so efficient. It is much easier to use via the wrappers.
  std::vector< int > getRobotEncoders(bool refreshEncoders = true) const;

  short getMotorVelocityLimit(short number) const;
  short getMotorAccelerationLimit(short number) const;

  void setMotorVelocityLimit(short number, short velocity);
  void setMotorAccelerationLimit(short number, short acceleration);
  /*!\brief Get the current force
   */
  short getForce(int axis);
  /*!\brief Get the axis controller type
   @return 0 for position controller, 1 for current controller
   */
  int getCurrentControllerType(int axis);

  void setRobotVelocityLimit(short velocity);
  /// Set the velocity of all motors together.
  /// This does not set the velocity of the TCP.
  void setRobotAccelerationLimit(short acceleration);

  void moveMotorByEnc(short number, int encoders, bool waitUntilReached = false, int waitTimeout = 0);
  void moveMotorBy(short number, double radianAngle, bool waitUntilReached = false, int waitTimeout = 0);

  void moveMotorToEnc(short number, int encoders, bool waitUntilReached = false, int encTolerance = 100,
                      int waitTimeout = 0);
  void moveMotorTo(short number, double radianAngle, bool waitUntilReached = false, int waitTimeout = 0);

  void waitForMotor(short number, int encoders, int encTolerance = 100, short mode = 0, int waitTimeout = 5000);
  void waitFor(TMotStsFlg status, int waitTimeout);

  /// Move to robot to given encoders.
  /// You can provide less values than the number of motors. In that case only the given ones will be moved.
  /// This can be usefull in cases where you want to move the robot but you don't want to move the gripper.
  void moveRobotToEnc(std::vector< int >::const_iterator start, std::vector< int >::const_iterator end,
                      bool waitUntilReached = false, int encTolerance = 100, int waitTimeout = 0);
  /// Move to robot to given encoders in the vector-container.
  /// This method is mainly provided for convenience. Catch by value (and not by reference) is intended to avoid
  /// nasty wrapping code.
  void moveRobotToEnc(std::vector< int > encoders, bool waitUntilReached = false, int encTolerance = 100,
                      int waitTimeout = 0);
  /// Move to robot to given target in the vector-container with the given velocity, acceleration and tolerance
  void moveRobotToEnc4D(std::vector< int > target, int velocity = 180, int acceleration = 1, int encTolerance = 100);

  void openGripper(bool waitUntilReached = false, int waitTimeout = 100);
  void closeGripper(bool waitUntilReached = false, int waitTimeout = 100);

  void freezeRobot();
  void freezeMotor(short number);
  void switchRobotOn();
  void switchRobotOff();
  void switchMotorOn(short number);
  void switchMotorOff(short number);

  /// Start a spline movement
  /// @param exactflag Set it to true if you want the position controller activated after the movement
  /// @param moreflag 0 = start moving more following, 1 = last or a single polynomial movement, 2 = do not start moving
  /// yet more following
  void startSplineMovement(bool exactflag, int moreflag = 1);

  ///
  /// Send one spline to the motor
  /// @param duration Duration has to be given in 10ms units
  void sendSplineToMotor(short number, short targetPosition, short duration, short p1, short p2, short p3, short p4);

  /// Send polynomials to all motors and start movement
  /// @param polynomial time, target pos and coefficients of the motor polynomials
  /// @param exactflag exactflag
  /// @param moreflag 0 = start moving more following, 1 = last or a single polynomial movement, 2 = do not start moving
  /// yet more following
  void setAndStartPolyMovement(std::vector< short > polynomial, bool exactflag, int moreflag);

  //!Read The Digital I/Os
  int readDigitalIO();
};

/******************************************************************************************************************/
#endif  //_KMLEXT_H_
/******************************************************************************************************************/
