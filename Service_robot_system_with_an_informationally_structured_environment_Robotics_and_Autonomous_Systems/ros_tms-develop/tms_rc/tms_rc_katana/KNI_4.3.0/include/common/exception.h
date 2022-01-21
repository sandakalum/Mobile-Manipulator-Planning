//
// C++ Interface: exception
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include <string>
#include "dllexport.h"

#ifdef HAVE_LIBEBT

#include <libebt/libebt.hh>
#include <libebt/libebt_util.hh>

struct ExceptionTag
{
};
typedef libebt::BacktraceContext< ExceptionTag, std::string > Context;

///
/// @defgroup exceptions Exceptions
/// @{
///

class DLLDIR Exception : public libebt::Backtraceable< ExceptionTag >, public std::exception
{
protected:
  const std::string _message;
  const int _error_number;

public:
  Exception(const std::string& message, const int error_number) throw()
    : libebt::Backtraceable< ExceptionTag >(), std::exception(), _message(message), _error_number(error_number)
  {
  }

  virtual ~Exception() throw()
  {
  }

  std::string message() const throw()
  {
    return _message;
  }
  const char* what() const throw()
  {
    return _message.c_str();
  }

  const int error_number() const throw()
  {
    return _error_number;
  }
};

///
/// @}
///

#else  // HAVE_LIBEBT

///
/// @defgroup exceptions Exceptions
/// @{
///

// The compiler should optimize that away
struct DLLDIR Context
{
  Context(const char*)
  {
  }
};

class DLLDIR Exception : public std::exception
{
protected:
  const std::string _message;
  const int _error_number;

public:
  Exception(const std::string& message, const int error_number) throw()
    : std::exception(), _message(message), _error_number(error_number)
  {
  }

  virtual ~Exception() throw()
  {
  }

  std::string message() const throw()
  {
    return _message;
  }
  const char* what() const throw()
  {
    return _message.c_str();
  }

  int error_number() const throw()
  {
    return _error_number;
  }
};

///
/// @}
///

#endif

#endif
