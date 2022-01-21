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

/*! \mainpage "Katana Native Interface Documentation"
 */

//--------------------------------------------------------------------------//
#ifndef _KMLBASE_H_
#define _KMLBASE_H_
//--------------------------------------------------------------------------//
#include "common/dllexport.h"

#include "KNI/cplBase.h"
#include "KNI/kmlCommon.h"
#include "KNI/kmlMotBase.h"
#include "KNI/kmlSctBase.h"

#include "KNI/cdlCOM.h"
#include "KNI/cdlCOMExceptions.h"

//--------------------------------------------------------------------------//
//!The old protocol is only supported up to K400 version 0.x.x
#define K400_OLD_PROTOCOL_THRESHOLD 3

#if !defined(BYTE_DECLARED)
#define BYTE_DECLARED
typedef unsigned char byte;  //!< type specification (8 bit)
#endif

//--------------------------------------------------------------------------//

#define TM_ENDLESS -1  //!< timeout symbol for 'endless' waiting

//--------------------------------------------------------------------------//

class CKatBase;  // katana
class CMotBase;  // motor
class CSctBase;  // sensor contoller

//--------------------------------------------------------------------------//
// CKatBase ----------------------------------------------------------------//
//--------------------------------------------------------------------------//

/*!	\brief	[GNL] general robot attributes
 */
struct TKatGNL
{
  byte adr;             //!< jumper adress
  char modelName[255];  //!< model name
};

/*!	\brief	[MFW] master firmware version/revision number
 */
struct TKatMFW
{
  byte ver;  //!< version
  byte rev;  //!< revision
};

/*!	\brief	[IDS] identification string
 */
struct TKatIDS
{
  byte strID[256];  //!< id string
};

/*!	\brief	[CTB] command table defined in the firmware
 */
struct TKatCTB
{
  byte cmdtbl[256];  //!< command table
};

/*!	\brief	[CBX] connector box
 */
struct TKatCBX
{
  bool inp[2];  //!< input: green & red LED
  bool out[2];  //!< output: green & red LED
};

/*!	\brief	[ECH] echo
 */
struct TKatECH
{
  byte echo;  //!< echo answer
};

/*! \brief Inverse Kinematics structure of the endeffektor
 *
 *	This structure describes the properties of the endeffector and it's used for the inverse
 *	kinematic calculations. An endeffector is a point where the attributes of this structure
 *	belong to. Please remember that the actual inverse kinematic calculations have been set
 *	up <b>only</b> for the Katana <b>6M</b> robot! So do not be astonished if you get strange
 *	behaviour with a Katana <b>5M</b>.
 */

struct TKatEFF
{
  double arr_segment[4];  //!< length of the Katana segments
};

//--------------------------------------------------------------------------//

/*!	\brief	Base Katana class
 *
 *	This class is the main object controlling the whole katana; to use it, it
 *	has to be initilized by using it's init function; those function expects
 *	a initilized protocol class, which in turn expects an initilized device!
 *	after the initialization, it does not mean that the coordinates (encoder
 *	values) of the motors have been set correctly; for that a calibration is
 *	needen; that calibration can be executed either by using the CKatana class
 *	in the 'kmlExt' module (which encapsulates this class) or by writing your
 *	own calibrations function..
 */
class DLLDIR CKatBase
{
protected:
  TKatGNL gnl;  //!< katana general
  TKatMFW mfw;  //!< master's firmware version/revision
  TKatIDS ids;  //!< ID string
  TKatCTB ctb;  //!< cmd table
  TKatCBX cbx;  //!< connector box
  TKatECH ech;  //!< echo

  TKatMOT mot;  //!< motors
  TKatSCT sct;  //!< sensor controllers
  TKatEFF eff;  //!< end effector

  CCplBase* protocol;     //!< protocol interface
  short mMasterVersion;   //!< master version of robot we are communicating with
  short mMasterRevision;  //!< master firmware revision

public:
  /*! \brief Get a pointer to the desired structure
	*/ const TKatGNL* GetGNL()
  {
    return &gnl;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatMFW* GetMFW()
  {
    return &mfw;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatIDS* GetIDS()
  {
    return &ids;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatCTB* GetCTB()
  {
    return &ctb;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatCBX* GetCBX()
  {
    return &cbx;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatECH* GetECH()
  {
    return &ech;
  }

  /*! \brief Get a pointer to the desired structure
	*/ const TKatMOT* GetMOT()
  {
    return &mot;
  }
  /*! \brief Get a pointer to the desired structure
	*/ const TKatSCT* GetSCT()
  {
    return &sct;
  }
  /*! \brief Get a pointer to the desired structure
	*/ TKatEFF* GetEFF()
  {
    return &eff;
  }

  CKatBase()
  {
  }
  /*!	\brief	destructor
  */
  virtual ~CKatBase()
  {
  }

  virtual bool init(const TKatGNL _gnl,  //!< general attributes
                    const TKatMOT _mot,  //!< motor attributes
                    const TKatSCT _sct,  //!< sensor controller attributes
                    const TKatEFF _eff,  //!< end effector attributes
                    CCplBase* _protocol  //!< desired protocol
                    );

  /*!\brief receive data
	*/ void recvMFW();
  /*!\brief receive data
	*/ void recvIDS();
  /*!\brief receive data
	*/ void recvCTB();
  /*!\brief receive data
	*/ void recvGMS();
  /*!\brief wait for motor on all motors
	*/ void waitFor(TMotStsFlg status, int waitTimeout, bool gripper);
  /*!\brief receive data
	*/ void recvECH();
  /*!\brief read all motor positions simultaneously
		 */ void recvMPS();
  /*!\brief get a handle of the protocol, used in CKatana*/
  CCplBase* getProtocol()
  {
    return protocol;
  }
  /*!\brief checks for a K300 or K400*/
  int checkKatanaType(int type);

  /*!	\brief	Get the master firmware of the robot we are communicating with
   *
   *	Get master firmware read at initialization time.
   */
  void getMasterFirmware(short* fw, short* rev);

  /*!\brief crash limits enable
	*/ void enableCrashLimits();
  /*!\brief crash limits disable
	*/ void disableCrashLimits();
  /*!\brief unblock robot after a crash
	*/ void unBlock();
  /*!\brief set collision limits
  *
  * //deprecated, use speed & position
  */
  void setCrashLimit(long idx, int limit);
  /*!\brief set collision position limits
  */
  void setPositionCollisionLimit(long idx, int limit);
  /*!\brief set collision speed limits
  */
  void setSpeedCollisionLimit(long idx, int limit);

  /// Start a spline movement
  /// @param exactflag Set it to 1 if you want the position controller activated after the movement, 0 otherwise, add 2
  /// to set motor6_follow (no gripper)
  /// @param moreflag 0 = start moving more following, 1 = last or a single polynomial movement, 2 = do not start moving
  /// yet more following
  void startSplineMovement(int exactflag, int moreflag = 1);

  /// Send polynomials to all motors and start movement
  /// @param polynomial time, target pos and coefficients of the motor polynomials
  /// @param exactflag exactflag
  /// @param moreflag 0 = start moving more following, 1 = last or a single polynomial movement, 2 = do not start moving
  /// yet more following
  void setAndStartPolyMovement(std::vector< short > polynomial, int exactflag, int moreflag);

  //!get digital I/O data from Katana400:
  int readDigitalIO();

  //!flush move buffers on all motors:
  int flushMoveBuffers();
};

/****************************************************************************/
#endif  //_KMLBASE_H_
/****************************************************************************/
