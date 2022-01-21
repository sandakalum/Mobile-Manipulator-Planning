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

#include "common/Timer.h"

#ifdef WIN32
#include <Windows.h>
#endif

namespace KNI
{
void Timer::Set(long timeout)
{
  _timeout = timeout;
}

#ifdef WIN32
void Timer::Start()
{
  _ct = clock();
}

long Timer::_ElapsedTime() const
{
  return static_cast< long >(1000.0 * (static_cast< double >((clock() - _ct)) / static_cast< double >(CLOCKS_PER_SEC)));
}

Timer::Timer() : _timeout(0), _ct(0)
{
}
Timer::Timer(long timeout) : _timeout(timeout), _ct(0)
{
}

#else

Timer::Timer() : _timeout(0), _ct()
{
}
Timer::Timer(long timeout) : _timeout(timeout), _ct()
{
}

void Timer::Start()
{
  gettimeofday(&_ct, 0);
}

long Timer::_ElapsedTime() const
{
  struct timeval end;
  gettimeofday(&end, 0);
  return (end.tv_sec * 1000 + end.tv_usec / 1000) - (_ct.tv_sec * 1000 + _ct.tv_usec / 1000);
}

#endif

void Timer::Set_And_Start(long timeout)
{
  Set(timeout);
  Start();
}

bool Timer::Elapsed() const
{
  if (_timeout <= 0)
    return false;  // (timeout <= 0) => INFINITE
  if (_ElapsedTime() < _timeout)
    return false;
  return true;
}
long Timer::ElapsedTime() const
{
  return _ElapsedTime();
}

void Timer::WaitUntilElapsed() const
{
  if (Elapsed())
    return;
  /// WARNING: possible race-condition in sleep() if
  /// it takes longer than 1ms between the check
  /// and the real sleep.
  sleep(_timeout - _ElapsedTime());
}

void sleep(long time)
{
  if (time <= 0)
    return;
#ifdef WIN32
  Sleep(time);
#else  // POSIX 1b
  timespec time2sleep;
  time2sleep.tv_sec = 0;
  time2sleep.tv_nsec = time * 1000 * 1000;  // SLEEP in milliseconds
  nanosleep(&time2sleep, NULL);
#endif
}
}
