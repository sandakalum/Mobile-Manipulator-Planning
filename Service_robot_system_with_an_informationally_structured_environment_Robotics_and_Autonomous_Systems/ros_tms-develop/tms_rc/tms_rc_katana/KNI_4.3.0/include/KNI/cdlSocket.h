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

#ifndef _CDLSOCKET_H_
#define _CDLSOCKET_H_

#include "common/dllexport.h"
#include "common/Timer.h"

#include "KNI/cdlBase.h"
#include "KNI/cdlCOMExceptions.h"

//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//
#include <windows.h>
#include <winsock.h>
#include <stdio.h>
//-------------------------------------------------------//
#else  // LINUX
//-------------------------------------------------------//
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cerrno>
#include <unistd.h>
//<tfromm date="22.05.2009">
//#include <string>
#include <cstring>
#include <cstdlib>
//</tfromm>
//-------------------------------------------------------//
#endif  // WIN32 else LINUX
        //-------------------------------------------------------//

//--------------------------------------------------------------------------//
/*!	\brief	Encapsulates the socket communication device.
 *
 *	This class is responsible for direct communication with the Katana robot or its simulation environment through
 *sockets.
 *	It builds the lowest layer for KNI communication and uses the system API functions to get access to the socket.
 *
 */

class DLLDIR CCdlSocket : public CCdlBase
{
private:
  //! IP Address of the Robot or simulation environment
  /*!Set to localhost or 127.0.0.1 if the simulation runs on the same machine*/
  char* _ipAddr;
  //!Port number of the KNI communication socket.
  int _port;
  //!Length of the message
  int _len;
//-------------------------------------------------------//
#ifdef WIN32
  //-------------------------------------------------------//
  // Windows specific socket handles:
  SOCKET _socketfd;
  SOCKADDR_IN _socketAddr;

//-------------------------------------------------------//
#else   // LINUX
  //-------------------------------------------------------//
  // Unix specific socket handles:
  //! File handler for the socket
  int _socketfd;
  //!Structure to fill in the socket communication parameteres
  struct sockaddr_in _socketAddr;
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
  /*! \brief	Constructs a CCdlSocket object
   *
   *	To this constructor the socket's AF_INET address (for platform independence) and port number have to be given as
   *parameters.
         *      An attempt to open a connection
   *	to the desired device will be tried and if successful, 'lastOP()'
   *	will return 'lopDONE', otherwise 'lopFAIL'.
   */
  CCdlSocket(char* adress, int port);

  /*! \brief	Destructs the object
   */
  virtual ~CCdlSocket();

  /*!	\brief	Sends data to the socket
   */
  virtual int send(const void* _buf, int _size);

  /*! \brief	Receives data from the socket
  */
  virtual int recv(void* _buf, int _size);

  /*! \brief	Terminates the socket connection
  */
  virtual int disconnect();
};

#endif  //_CDLSOCKET_H_
