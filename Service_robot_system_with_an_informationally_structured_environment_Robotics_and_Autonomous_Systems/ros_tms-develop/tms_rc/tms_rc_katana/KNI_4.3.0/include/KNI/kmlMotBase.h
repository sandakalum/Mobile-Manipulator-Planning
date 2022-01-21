//
// C++ Interface: MotBase
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef KMLMOTBASE_H
#define KMLMOTBASE_H

#include "common/exception.h"
#include "common/dllexport.h"

#include "KNI/kmlCommon.h"
#include "KNI/cplBase.h"

#include <vector>

class CKatBase;  // forward declaration
class CMotBase;  // forward declaration

/****************************************************************************/
// CMotBase ----------------------------------------------------------------//
/****************************************************************************/

/*!	\brief	motor description (partly)
 */
struct TMotDesc
{
  byte slvID;  //!< slave number
};

/*!	\brief	[MOT] every motor's attributes
 */
struct TKatMOT
{
  short cnt;       //!< count of motors
  CMotBase* arr;   //!< array of motors
  TMotDesc* desc;  //!< description[]
};

/*!	\brief	command flags
 */
enum TMotCmdFlg
{
  MCF_OFF = 0,               //!< set the motor off
  MCF_CALIB = 4,             //!< calibrate
  MCF_FREEZE = 8,            //!< freeze the motor
  MCF_ON = 24,               //!< set the motor on
  MCF_CLEAR_MOVEBUFFER = 32  //!< clear the movebuffer
};

/*!	\brief	status flags
 */
enum TMotStsFlg
{
  MSF_MECHSTOP = 1,     //!< mechanical stopper reached, new: unused (default value)
  MSF_MAXPOS = 2,       //!< max. position was reached, new: unused
  MSF_MINPOS = 4,       //!< min. position was reached, new: calibrating
  MSF_DESPOS = 8,       //!< in desired position, new: fixed, state holding
  MSF_NORMOPSTAT = 16,  //!< trying to follow target, new: moving (polymb not full)
  MSF_MOTCRASHED = 40,  //!< motor has crashed, new: collision
  MSF_NLINMOV = 88,     //!< non-linear movement ended, new: poly move finished
  MSF_LINMOV = 152,     //!< linear movement ended, new: moving poly, polymb full
  MSF_NOTVALID = 128    //!< motor data not valid
};
enum TSearchDir
{  //!< search direction for the meachanical stopper
  DIR_POSITIVE,
  DIR_NEGATIVE
};

//--------------------------------------------------------------------------//

/*!	\brief	[GNL] motor generals
 */
struct TMotGNL
{
  CKatBase* own;  //!< parent robot
  byte SID;       //!< slave ID
};

/*!	\brief	[SFW] slave firmware
 */
struct TMotSFW
{
  byte version;     //!< firmware version number
  byte subversion;  //!< firmware subversion number
  byte revision;    //!< firmware revision number
  byte type;        //!< controller type
  byte subtype;     //!< slave subtype
};

/*!	\brief	[APS] actual position
 */
struct TMotAPS
{
  TMotCmdFlg mcfAPS;  //!< motor command flag
  short actpos;       //!< actual position
};

/*!	\brief	[TPS] target position
 */
struct TMotTPS
{
  TMotCmdFlg mcfTPS;  //!< motor command flag
  short tarpos;       //!< target position
};

/*!	\brief	[SCP] static controller parameters
 */
struct TMotSCP
{
  //--------------- Motor old parameters -------------------------------//
  //
  byte maxppwm;  //!< max. val for pos. voltage
  byte maxnpwm;  //!< max. val for neg. voltage; pos!
  byte kP;       //!< prop. factor of pos comp
  byte kI;       //!< not yet active
  byte kD;       //!< derivate factor of pos comp
  byte kARW;     //!< not yet active
  // byte		kSpeed;		//!< prop. factor of speed limit comp
  byte kP_speed;  //!< Proportional factor of the speed compensator
  byte kI_speed;  //!< Integral factor of the speed compensator
  byte kD_speed;  //!< Derivative factor of the speed compensator

  //--------------- Motor new parameters -------------------------------//
  //
  byte maxppwm_nmp;         //!< Max. value for positive voltage (0 => 0%, +70 => 100%)
  byte maxnpwm_nmp;         //!< Max. value for negative voltage (0 => 0%, +70 => 100%)
  byte kspeed_nmp;          //!< Proportional factor of speed compensator
  byte kpos_nmp;            //!< Proportional factor of position compensator
  byte kI_nmp;              //!< Integral factor (1/kI) of control output added to the final control output
  int crash_limit_nmp;      //!< Limit of error in position
  int crash_limit_lin_nmp;  //!< Limit of error in position in linear movement
};

/*!	\brief	[DYL] dynamic limits
 */
struct TMotDYL
{
  //--------------- Motor old parameters -------------------------------//
  //
  byte maxaccel;    //!< max acceleration
  byte maxdecel;    //!< max deceleration
  short minpos;     //!< not yet active
  short maxpspeed;  //!< max. allowed forward speed
  short maxnspeed;  //!< max. allowed reverse speed; pos!
  // byte		maxpcurr;	// no more active
  // byte		maxncurr;	// no more active
  byte maxcurr;  //!< max current
  byte actcurr;  //!< actual current

  //--------------- Motor new parameters -------------------------------//
  //
  byte maxaccel_nmp;    //!< Maximal acceleration and deceleration
  short maxpspeed_nmp;  //!< Max. allowed forward speed
  short maxnspeed_nmp;  //!< Max. allowed reverse speed
  byte maxcurr_nmp;     //!< set the maximal current
};

/*!	\brief	[PVP] position, velocity, pulse width modulation
 */
struct TMotPVP
{
  TMotStsFlg msf;  //!< motor status flag
  short pos;       //!< position
  short vel;       //!< velocity
  byte pwm;        //!< pulse with modulation
};

/*!	\brief	[ENL] limits in encoder values (INTERNAL STRUCTURE!)
 */
struct TMotENL
{
  int enc_range;      //!< motor's range in encoder values
  int enc_minpos;     //!< motor's minimum position in encoder values
  int enc_maxpos;     //!< motor's maximum position in encoder values
  int enc_per_cycle;  //!< number of encoder units needed to complete 360 degrees;
  int enc_tolerance;  //!< encoder units of tolerance to accept that a position has been reached
};

/*!	\brief	Calibration structure for single motors.
 */
struct TMotCLB
{
  bool enable;  //!< enable/disable
  short order;  //!< order in which this motor will be calibrated. range: 0..5

  TSearchDir dir;  //!< search direction for mech. stopper
  TMotCmdFlg mcf;  //!< motor flag after calibration

  int encoderPositionAfter;
  bool isCalibrated;

  TMotDYL dyl;
  TMotSCP scp;
};

/*!	\brief	Initial motor parameters
 */
struct TMotInit
{
  int encoderOffset;
  int encodersPerCycle;
  double angleOffset;
  double angleRange;
  int rotationDirection;

  // calculated ones:
  double angleStop;
};

//--------------------------------------------------------------------------//

/*!	\brief	Motor class
 *

 *	This class allows to control one motor; to control a motor it has to be
 *	initialized by using the init function. And the usage the internal allocated
 *	resources should be deallocated by using the 'free' method.

 */
class DLLDIR CMotBase
{
  friend class CKatBase;

protected:
  TMotGNL gnl;                     //!< motor generals
  TMotAPS aps;                     //!< actual position
  TMotTPS tps;                     //!< target position
  TMotSCP scp;                     //!< static controller parameters
  TMotDYL dyl;                     //!< dynamic limits
  TMotPVP pvp;                     //!< reading motor parameters
  TMotSFW sfw;                     //!< slave firmware
  TMotCLB _calibrationParameters;  //!< calibration structure
  TMotENL _encoderLimits;          //!< motor limits in encoder values
  TMotInit _initialParameters;
  bool freedom;  //!< if it is set, it will move on a parallel movement
  bool blocked;  //!< true if the motor was blocked due to a crash of the robot

public:
  const TMotGNL* GetGNL()
  {
    return &gnl;
  }
  const TMotAPS* GetAPS()
  {
    return &aps;
  }
  const TMotTPS* GetTPS()
  {
    return &tps;
  }
  const TMotSCP* GetSCP()
  {
    return &scp;
  }
  const TMotDYL* GetDYL()
  {
    return &dyl;
  }
  const TMotPVP* GetPVP()
  {
    return &pvp;
  }
  const TMotSFW* GetSFW()
  {
    return &sfw;
  }
  const TMotCLB* GetCLB()
  {
    return &_calibrationParameters;
  }

  const TMotInit* GetInitialParameters()
  {
    return &_initialParameters;
  }
  int GetEncoderTolerance()
  {
    return _encoderLimits.enc_tolerance;
  }
  int GetEncoderMinPos()
  {
    return _encoderLimits.enc_minpos;
  }  //!<Returns the min Position of the Encoder
  int GetEncoderMaxPos()
  {
    return _encoderLimits.enc_maxpos;
  }  //!<Returns the max Position of the Encoder
  int GetEncoderRange()
  {
    return _encoderLimits.enc_range;
  }  //!<Returns Encoder Range of the Encoder

  /*! \brief Get the value of the freedom property
	*/ bool GetFreedom()
  {
    return freedom;
  }
  /*! \brief Get the value of the blocked property
	*/ bool GetBlocked()
  {
    return blocked;
  }

protected:
  CCplBase* protocol;  //!< protocol interface

public:
  virtual ~CMotBase()
  {
  }  // destructor

  bool init(CKatBase* _own, const TMotDesc _motDesc, CCplBase* protocol);

  /*!\brief send data
	*/ void sendAPS(const TMotAPS* _aps);
  /*!\brief send data
	*/ void sendTPS(const TMotTPS* _tps);

  /*!\brief receive data
	*/ void recvPVP();
  /*!\brief receive data
	*/ void recvSFW();

  void setSCP(TMotSCP _scp)
  {
    scp = _scp;
  }
  void setDYL(TMotDYL _dyl)
  {
    dyl = _dyl;
  }

  void setInitialParameters(double angleOffset, double angleRange, int encodersPerCycle, int encoderOffset,
                            int rotationDirection);
  void setCalibrationParameters(bool doCalibration, short order, TSearchDir direction, TMotCmdFlg motorFlagAfter,
                                int encoderPositionAfter);
  void setCalibrated(bool calibrated);

  void setTolerance(int tolerance);

  /*!\brief check limits in encoder values
  */
  bool checkAngleInRange(double angle);
  bool checkEncoderInRange(int encoder);

  /*!	\brief	Increments the motor specified by an index postion in encoder units.
   */
  void inc(int dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Decrements the motor specified by an index postion in encoder units.
   */
  void dec(int dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Moves the motor specified by an index to a given target position in encoder units.
   */
  void mov(int tar, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);

  /*!	\brief	Waits until the Motor has reached the given targen position
   */
  void waitForMotor(int tar, int encTolerance = 100, short mode = 0, int waitTimeout = TM_ENDLESS);

  /*!	\brief	Increments the motor specified by an index postion in degrees.
   */
  void incDegrees(double dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Decrements the motor specified by an index postion in degrees.
   */
  void decDegrees(double dif, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);
  /*!	\brief	Moves the motor specified by an index to a given target position in degrees.
   */
  void movDegrees(double tar, bool wait = false, int tolerance = 100, long timeout = TM_ENDLESS);

  /*!	\brief	unblock the motor.
   */
  void resetBlocked();

  ///
  /// Send one spline to the motor
  /// @param duration Duration has to be given in 10ms units
  void sendSpline(short targetPosition, short duration, short p1, short p2, short p3, short p4);

  ///
  /// Set speed limits
  void setSpeedLimits(short positiveVelocity, short negativeVelocity);
  void setSpeedLimit(short velocity)
  {
    setSpeedLimits(velocity, velocity);
  }

  ///
  /// Set the acceleration limits
  void setAccelerationLimit(short acceleration);

  ///
  /// Set the PWM limits (for the drive controller)
  void setPwmLimits(byte maxppwm, byte maxnpwm);

  ///
  /// Set the controller parameters
  void setControllerParameters(byte kSpeed, byte kPos, byte kI);

  ///
  /// Set the crash limit
  void setCrashLimit(int limit);
  /// Set the crash limit linear
  void setCrashLimitLinear(int limit_lin);
  /// Set the collision limit
  void setSpeedCollisionLimit(int limit);
  /// Set the collision limit
  void setPositionCollisionLimit(int limit);

  ///
  /// Get parameters or limits
  /// @param subcommand	255-249;245, see katana user manual chapter 8 firmware commands for details
  /// @param R1	pointer to store first byte of answer
  /// @param R2	pointer to store second byte of answer
  /// @param R3	pointer to store third byte of answer
  void getParameterOrLimit(int subcommand, byte* R1, byte* R2, byte* R3);
};

#endif
