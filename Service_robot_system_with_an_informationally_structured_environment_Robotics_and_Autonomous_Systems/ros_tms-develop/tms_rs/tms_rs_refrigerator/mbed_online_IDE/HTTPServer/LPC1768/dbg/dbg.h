
/*
Copyright (c) 2010 Donatien Garnier (donatiengar [at] gmail [dot] com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/** \file
Debugging helpers header file
*/

//#ifdef DBG_H
//#define DBG_H

#ifdef __LWIP_DEBUG
#define __DEBUG
#endif

/*!
  \def __DEBUG
  To define to enable debugging in one file
*/

#ifdef __DEBUG

#ifndef __DEBUGSTREAM
#define __DEBUGSTREAM

class DebugStream
{
public:
  static void debug(const char* format, ...);
  static void release();
  static void breakPoint(const char* file, int line);

private:
};

#undef DBG
#undef DBG_END
#undef BREAK

/// Debug output (if enabled), same syntax as printf, with heading info
#define DBG(...)                                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    DebugStream::debug("[%s:%s@%d] ", __FILE__, __FUNCTION__, __LINE__);                                               \
    DebugStream::debug(__VA_ARGS__);                                                                                   \
  } while (0);

/// Debug output (if enabled), same syntax as printf, no heading info
#define DBGL(...)                                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    DebugStream::debug(__VA_ARGS__);                                                                                   \
  } while (0);
#define DBG_END DebugStream::release

/// Break point usin serial debug interface (if debug enbaled)
#define BREAK() DebugStream::breakPoint(__FILE__, __LINE__)
#endif

#else
#undef DBG
#undef DBG_END
#undef BREAK
#define DBG(...)
#define DBG_END()
#define BREAK()
#endif

#ifdef __LWIP_DEBUG
#ifndef __SNPRINTF
#define __SNPRINTF
#include "mbed.h"

// int snprintf(char *str, int size, const char *format, ...);
#endif
#endif

#ifdef __LWIP_DEBUG
#undef __DEBUG
#endif

//#endif
