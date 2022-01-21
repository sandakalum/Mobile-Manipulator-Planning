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
#ifndef _CPLSERIAL_H_
#define _CPLSERIAL_H_
/******************************************************************************************************************/
#include "common/dllexport.h"
#include "common/exception.h"
#include "KNI/cplBase.h"
#include "KNI/cdlCOMExceptions.h"

#define NUMBER_OF_RETRIES_SEND 3  // Number of retries after a failing in the Communication
#define NUMBER_OF_RETRIES_RECV 3  // Number of retries after a failing in the Communication
/******************************************************************************************************************/

//!defines the error flag number
const int KATANA_ERROR_FLAG = 192;
///
/// @addtogroup exceptions
/// @{
///

///
/// CRC check for the answer package failed
class WrongCRCException : public Exception
{
public:
  WrongCRCException() throw() : Exception("CRC check failed", -20)
  {
  }
};

///
/// Exception reported by the firmware
class FirmwareException : public Exception
{
protected:
  int _axis_number;    //!< axis number, if any
  char _command_char;  //!< the command that caused the error
public:
  FirmwareException(const std::string& error, const int error_number, const int axis, const char command) throw()
    : Exception("FirmwareException : '" + error + "'", error_number), _axis_number(axis), _command_char(command)
  {
  }
  int axis_number() const throw()
  {
    return _axis_number;
  }
  char command_char() const throw()
  {
    return _command_char;
  }
};

///
/// @}
///

/*!	\brief	Header of a communication packet
 */
struct THeader
{
  byte size;       //!< header size
  byte data[256];  //!< data part: 16x zero, 1x one, 1x katadr
};

/*!	\brief	Communication packet
 */
struct TPacket
{
  byte send_sz;  //!< send size of the packet
  byte read_sz;  //!< read size of the packet
};

//----------------------------------------------------------------------------------------------------------------//

/*!	\brief	Base class of two different serial protocols
 */
class DLLDIR CCplSerial : public CCplBase
{
protected:
  THeader hdr;       //!< header
  TPacket cmd[256];  //!< command table

  byte send_buf[256];  //!< sending buffer
  byte read_buf[256];  //!< receive buffer

protected:
  virtual bool load_tbl() = 0;                     //!< Loads the command table from the robot's firmware.
  virtual void defineProtocol(byte _kataddr) = 0;  //!< Defines the protocol's attributes.
};

//----------------------------------------------------------------------------------------------------------------//

/*! \brief	Implement the Serial-Zero protocol
 */

// class DLLDIR CCplSerialZero : public CCplSerial {
//
// protected:
// 	virtual bool load_tbl();					//!< Loads the command table from the robot's firmware.
// 	virtual void defineProtocol(byte _kataddr);	//!< Defines the protocol's attributes.
//
// public:
// 	/*!	\brief	Initializing function
// 	 *
// 	 *	Init the protocols basic attributes.
// 	 */
// 	virtual bool init(CCdlBase* _device, byte _kataddr = 24);
//
//
// 	/*!	\brief	Communication function
// 	 *
// 	 *	Sends a communications packet and receives one from the robot.
// 	 */
// 	virtual TRetCOMM comm(const byte* _pack, byte* _buf, byte* _size);
// };

//----------------------------------------------------------------------------------------------------------------//

/*! \brief	Implement the Serial-CRC protocol
 */
class DLLDIR CCplSerialCRC : public CCplSerial
{
protected:
  virtual bool load_tbl();                     //!< Loads the command table from the robot's firmware.
  virtual void defineProtocol(byte _kataddr);  //!< Defines the protocol's attributes.
  virtual void send(byte* send_buf, byte write_sz, short retries = 3);  // Sends a packet.
  virtual void recv(byte* read_buf, byte read_sz, byte* size);          // Receives the packet and checks the CRC.

public:
  /*!	\brief	Initializing function
   *
   *	Init the protocols basic attributes.
   */
  virtual bool init(CCdlBase* _device, byte _kataddr = 24);

  /*!	\brief	Communication function
   *
   *	Sends a communications packet and receives one from the robot.
   */
  virtual void comm(const byte* pack, byte* buf, byte* size);

  /*!	\brief	Get the master firmware of the robot we are communicating with
   *
   *	Get master firmware read at initialization time.
   */
  virtual void getMasterFirmware(short* fw, short* rev);
};

/******************************************************************************************************************/
#endif  //_CPLSERIALZERO_H_
/******************************************************************************************************************/
