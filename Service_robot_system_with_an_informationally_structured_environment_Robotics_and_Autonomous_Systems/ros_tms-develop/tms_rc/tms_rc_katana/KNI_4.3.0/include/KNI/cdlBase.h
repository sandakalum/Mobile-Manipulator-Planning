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

/****************************************************************************/
#ifndef _CDLBASE_H_
#define _CDLBASE_H_
/****************************************************************************/

#if !defined(BYTE_DECLARED)
#define BYTE_DECLARED
typedef unsigned char byte;  //!< type specification (8 bit)
#endif

//--------------------------------------------------------------------------//

#include "common/dllexport.h"

//--------------------------------------------------------------------------//

/*! \brief Abstract base class for devices
 *
 * This class is the base abstract class for devices; the abbreviation 'cdl'
 * stands for 'Communication Device Layer'. By inheriting from this class
 * different communication devices such a USB or a COM port can be handled
 * easier.
 */

class DLLDIR CCdlBase
{
public:
  /*! \brief Pure function to send data
   *
   * This function is pure and should always be overwritten by classes
   * inheriting from 'CCdlBase'. As the name proposes the function should
   * contain a sending behavour from the device.
   */
  virtual int send(const void* _buf, int _sz) = 0;
  /*! \brief Pure function to receive data
   *
   * This function is pure and should always be overwritten by classes in-
   * heriting from 'CCdlBase'. As the name proposes the function should
   * contain a sending behavour from the device.
   */
  virtual int recv(void* _buf, int _sz) = 0;
  /*! \brief destructor
   *
   * This class is only an interface
   */
  virtual ~CCdlBase(){};
};

/****************************************************************************/
#endif  //_CDLBASE_H_
/****************************************************************************/
