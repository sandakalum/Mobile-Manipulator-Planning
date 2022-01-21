//
// C++ Interface: SctBase
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef KMLSCTBASE_H
#define KMLSCTBASE_H

#include "common/dllexport.h"

#include "KNI/kmlCommon.h"
#include "KNI/cplBase.h"

class CKatBase;  // forward declaration
class CSctBase;  // forward declaration

/****************************************************************************/
// CSctBase ----------------------------------------------------------------//
/****************************************************************************/

/*!	\brief sensor controller description (partly)
 */
struct TSctDesc
{
  byte ctrlID;       //!< controller number (ID)
  short sens_res;    //!< resolution: 8/12 bit
  short sens_count;  //!< count of sensors
};

/*!	\brief	[SCT] every sens ctrl's attributes
 */
struct TKatSCT
{
  short cnt;       //!< count of sens ctrl's
  CSctBase* arr;   //!< array of sens ctrl's
  TSctDesc* desc;  //!< description[]
};

/*!	\brief	[GNL] controller generals
 */
struct TSctGNL
{
  CKatBase* own;  //!< parent robot
  byte SID;       //!< slave ID
  short res;      //!< resolution: 8/12 bit
};

/*!	\brief	[DAT] sensor data
 */
struct TSctDAT
{
  short cnt;   //!< count of sensors
  short* arr;  //!< sensor data
};

//--------------------------------------------------------------------------//

/*!	\brief	Sensor Controller class
 *
 *	By using this class you can get access to the sensor data; to do so you
 *	should (after initialization) call 'recvDat()' to updated the internal
 *	'TSctDAT dat' structure; after the updated you can read out the values
 *	by using the 'GetDAT()' function, which will return a constant pointer
 *	to the internal 'dat' structure.
 */
class DLLDIR CSctBase
{
  friend class CKatBase;

protected:
  TSctGNL gnl;  //!< controller generals
  TSctDAT dat;  //!< sensor data

public:
  const TSctGNL* GetGNL()
  {
    return &gnl;
  }
  const TSctDAT* GetDAT()
  {
    return &dat;
  }

protected:
  CCplBase* protocol;  //!< protocol interface

public:
  virtual ~CSctBase()
  {
  }  // destructor

  bool init(CKatBase* _own, const TSctDesc _sctDesc, CCplBase* protocol);

  /*!\brief receive data
  */
  void recvDAT();
};

#endif
