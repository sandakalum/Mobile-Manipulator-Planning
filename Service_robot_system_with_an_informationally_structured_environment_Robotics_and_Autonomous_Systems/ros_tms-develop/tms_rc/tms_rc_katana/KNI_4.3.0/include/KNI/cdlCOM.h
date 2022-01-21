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

#ifndef _CDLCOM_H_
#define _CDLCOM_H_

#include "common/dllexport.h"

#include "KNI/cdlBase.h"
#include "KNI/cdlCOMExceptions.h"

//<tfromm date="22.05.2009">
//#include <string>
#include <cstring>
//</tfromm>

//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//
#include <windows.h>
//-------------------------------------------------------//
#else  // LINUX
//-------------------------------------------------------//
#include <termios.h>
#include <fcntl.h>
#include <cerrno>
//-------------------------------------------------------//
#endif  // WIN32 else LINUX
        //-------------------------------------------------------//

/*! \brief	This structrue stores the attributes for a
 *			serial port device.
 */

struct TCdlCOMDesc
{
  int port;    //!<	serial port number
  int baud;    //!<	baud rate of port
  int data;    //!<	data bit
  int parity;  //!<	parity bit
  int stop;    //!<	stop bit
  int rttc;    //!<	read  total timeout
  int wttc;    //!<	write total timeout
};

//--------------------------------------------------------------------------//

/*!	\brief	Encapsulates the serial port device.
 *
 *	This class is responsible for direct communication with the serial port
 *	device. It builds the lowest layer for communication and uses the system
 *	API functions to get access the to the device.
 */

class DLLDIR CCdlCOM : public CCdlBase
{
private:
  std::string _deviceName;

protected:
  TCdlCOMDesc _ccd;  //!< Stores the attributes of the serial port device.

//-------------------------------------------------------//
#ifdef WIN32
  //-------------------------------------------------------//
  HANDLE _prtHdl;     //!< port handle
  COMMTIMEOUTS _oto;  //!< old timeouts
//-------------------------------------------------------//
#else   // LINUX
  //-------------------------------------------------------//
  int _prtHdl;          //!< port handle
  struct termios _oto;  //!< old timeouts
                        //-------------------------------------------------------//
#endif  // WIN32 else LINUX
  //-------------------------------------------------------//

protected:
  /*! \brief	Converts an integer to a char.
   */
  static char digit(const int _val)
  {
    return (char)((int)'0' + _val);
  }

public:
  /*! \brief	Construct a CCdlCOM class
   *
   *	To this constructor a 'TCdlCOMDesc' parameter has to be given, which
   *	describes the desired serial port. An attempt to open a connection
   *	to the desired device will be tried.
   */
  CCdlCOM(TCdlCOMDesc ccd);

  /*! \brief	Destructs the class
	 */ virtual ~CCdlCOM();

  /*!	\brief	Sends data to the device
	 */ virtual int send(const void* buf, int size);

  /*! \brief	Receives data from the device
	*/ virtual int recv(void* buf, int size);
};

#endif  //_CDLCOM_H_
