//
// C++ Interface: kmlCommon
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef KMLCOMMON_H
#define KMLCOMMON_H

#include "common/exception.h"
#include <string>

#define TM_ENDLESS -1  //!< timeout symbol for 'endless' waiting

#if !defined(BYTE_DECLARED)
#define BYTE_DECLARED
typedef unsigned char byte;  //!< type specification (8 bit)
#endif

///
/// @addtogroup exceptions
/// @{
///

///
/// Slave error occurred
/// \note error_number=-31
class SlaveErrorException : public Exception
{
public:
  SlaveErrorException() throw() : Exception("Internal Slave Error", -31)
  {
  }
};

///
/// There was an error while reading a parameter from the robot
/// \note error_number=-32
class ParameterReadingException : public Exception
{
public:
  ParameterReadingException(const std::string &para) throw()
    : Exception("Reading '" + para + "' parameters failed", -32)
  {
  }
};

///
/// The data you wanted to send to the robot was invalid
/// \note error_number=-33
class ParameterWritingException : public Exception
{
public:
  ParameterWritingException(const std::string &para) throw()
    : Exception("Writing '" + para + "' parameters failed", -33)
  {
  }
};

///
/// The given parameter was wrong
/// \note error_number=-34
class WrongParameterException : public Exception
{
public:
  WrongParameterException(const std::string &para) throw() : Exception("Wrong parameters '" + para + "' set", -34)
  {
  }
};

///
/// The encoders for the given motor were out of range
/// \note error_number=-35
class MotorOutOfRangeException : public Exception
{
public:
  MotorOutOfRangeException() throw() : Exception("Encoders for motor are out of range", -35)
  {
  }
};

///
/// The timeout elapsed for the given motor and target position
/// \note error_number=-36
class MotorTimeoutException : public Exception
{
public:
  MotorTimeoutException() throw() : Exception("Motor timeout", -36)
  {
  }
};

///
/// The requested motor crashed during the movement
/// \note error_number=-37
class MotorCrashException : public Exception
{
public:
  MotorCrashException() throw() : Exception("Motor crashed", -37)
  {
  }
};

///
/// @}
///

#endif
