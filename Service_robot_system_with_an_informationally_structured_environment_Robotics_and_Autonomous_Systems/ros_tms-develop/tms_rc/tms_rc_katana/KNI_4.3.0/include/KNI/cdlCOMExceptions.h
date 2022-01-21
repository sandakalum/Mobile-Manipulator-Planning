//
// C++ Interface: cdlCOMExceptions
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef _CDLCOMEXCEPTIONS_H_
#define _CDLCOMEXCEPTIONS_H_

#include "common/exception.h"
#include <string>

//!Error codes in error handling strings
enum
{
  ERR_FAILED = -1,
  ERR_INVALID_ARGUMENT = -2,
  ERR_STATE_MISMATCH = -3,
  ERR_TYPE_MISMATCH = -4,
  ERR_RANGE_MISMATCH = -5,
  ERR_AXIS_HEARTBEAT = -6,
  ERR_AXIS_OPERATIONAL = -7,
  ERR_AXIS_MOVE = -8,
  ERR_AXIS_MOVE_POLY = -9,
  ERR_AXIS_COLLISION = -10,
  ERR_AXIS_ANY = -11,
  ERR_CRC = -12,
  ERR_PERIPHERAL = -13,
  ERR_MESSAGE = 192,
  ERR_MESSAGE_STRING = 193
};

///
/// @addtogroup exceptions
/// @{
///

///
/// Failed to open the serial communication device
/// \note error_number=-10
/// \note Linux only: You get also the direct error message from the system
class CannotOpenPortException : public Exception
{
public:
  CannotOpenPortException(const std::string &port, const std::string os_msg) throw()
    : Exception("Cannot open port '" + port + "': " + os_msg, -10)
  {
  }
};

///
/// Could not set or get the attributes for the given serial communication device
/// \note error_number=-11
class CannotGetSetPortAttributesException : public Exception
{
public:
  CannotGetSetPortAttributesException(const std::string &port) throw()
    : Exception("Cannot get/set attributes on '" + port + "'", -11)
  {
  }
};

///
/// The port was not open
/// \note error_number=-12
class PortNotOpenException : public Exception
{
public:
  PortNotOpenException(const std::string &port) throw() : Exception("Port '" + port + "' not open", -12)
  {
  }
};

///
/// Reading from the serial communication device failed
/// \note error_number=-13
/// \note Linux only: You get also the direct error message from the system
class DeviceReadException : public Exception
{
public:
  DeviceReadException(const std::string &port, const std::string os_msg) throw()
    : Exception("Read failure on port '" + port + "': " + os_msg, -13)
  {
  }
};

///
/// Writing to the serial communication device failed
/// \note error_number=-14
/// \note Linux only: You get also the direct error message from the system
class DeviceWriteException : public Exception
{
public:
  DeviceWriteException(const std::string &port, const std::string os_msg) throw()
    : Exception("Write failure on port '" + port + "': " + os_msg, -14)
  {
  }
};

///
/// This exception is the base for the WriteNotComplete and ReadNotCompleteException
///
class ReadWriteNotCompleteException : public Exception
{
public:
  ReadWriteNotCompleteException(const std::string &errstr, const int error_number) throw()
    : Exception(errstr, error_number)
  {
  }
};

///
/// Not all bytes could be written to the serial communication device
/// \note error_number=-15
class WriteNotCompleteException : public ReadWriteNotCompleteException
{
public:
  WriteNotCompleteException(const std::string &port) throw()
    : ReadWriteNotCompleteException("Cannot write all date to '" + port + "'", -15)
  {
  }
};

///
/// The Katana didn't answer correctly within the given timeout
/// \note error_number=-16
class ReadNotCompleteException : public ReadWriteNotCompleteException
{
public:
  ReadNotCompleteException(const std::string &port) throw()
    : ReadWriteNotCompleteException("Cannot read all data from '" + port + "'", -16)
  {
  }
};

///
/// The Katana returned an error string
/// \note error_number=-16
class ErrorException : public Exception
{
public:
  ErrorException(const std::string &error) throw() : Exception(error, -20)
  {
  }
};

///
/// @}
///

#endif
