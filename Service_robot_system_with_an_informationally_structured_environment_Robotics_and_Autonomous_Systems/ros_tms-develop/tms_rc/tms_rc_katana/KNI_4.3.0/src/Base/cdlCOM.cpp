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

#include "KNI/cdlCOM.h"

#include "common/Timer.h"

#include <unistd.h>

#ifdef WIN32

CCdlCOM::CCdlCOM(TCdlCOMDesc ccd) : _deviceName(""), _ccd(), _prtHdl(INVALID_HANDLE_VALUE), _oto()
{
  DCB commDCB;       // COM port parameters
  COMMTIMEOUTS nto;  // new timeouts
  char comX[5];
  char dcb[35];
  int i, d;

  std::string deviceName;
  HANDLE prtHdl;
  COMMTIMEOUTS oto;

  strncpy_s(comX, "COM ", 5);
  comX[3] = digit(ccd.port);
  deviceName = comX;
  prtHdl = CreateFile(comX, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING,
                      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH | FILE_FLAG_NO_BUFFERING, 0);

  if (prtHdl == INVALID_HANDLE_VALUE)
  {
    throw CannotOpenPortException(_deviceName, "info from win32-system not yet available");
  }

  FillMemory(&commDCB, sizeof(commDCB), 0);
  commDCB.DCBlength = sizeof(commDCB);
  strncpy_s(dcb, "baud=       parity=  data=  stop= ", 35);
  for (i = 5, d = 100000; d >= 1; d = d / 10)
  {
    if (d <= ccd.baud)
    {
      dcb[i++] = digit((ccd.baud / d) % 10);
    }
  }
  dcb[19] = ccd.parity;
  dcb[26] = digit(ccd.data);
  dcb[33] = digit(ccd.stop);
  if (!BuildCommDCB(dcb, &commDCB))
  {
    CloseHandle(prtHdl);
    throw CannotGetSetPortAttributesException(_deviceName);
  }

  commDCB.fAbortOnError = false;
  commDCB.fInX = false;
  commDCB.fOutX = false;
  commDCB.fOutxCtsFlow = false;
  if (!SetCommState(prtHdl, &commDCB))
  {
    CloseHandle(prtHdl);
    throw CannotGetSetPortAttributesException(_deviceName);
  }

  PurgeComm(prtHdl, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

  GetCommTimeouts(prtHdl, &oto);
  nto.ReadIntervalTimeout = MAXDWORD;
  nto.ReadTotalTimeoutMultiplier = 0;
  nto.ReadTotalTimeoutConstant = ccd.rttc;
  nto.WriteTotalTimeoutMultiplier = 0;
  nto.WriteTotalTimeoutConstant = ccd.wttc;
  if (!SetCommTimeouts(prtHdl, &nto))
  {
    CloseHandle(prtHdl);
    throw CannotGetSetPortAttributesException(_deviceName);
  }

  // Everything done, now we can change the state
  _prtHdl = prtHdl;
  _deviceName = deviceName;
  _ccd = ccd;
  _oto = oto;
}

CCdlCOM::~CCdlCOM()
{
  if (_prtHdl == INVALID_HANDLE_VALUE)
    return;

  FlushFileBuffers(_prtHdl);
  SetCommTimeouts(_prtHdl, &_oto);

  CloseHandle(_prtHdl);
}

int CCdlCOM::send(const void* buf, int size)
{
  if (_prtHdl == INVALID_HANDLE_VALUE)
  {
    throw PortNotOpenException(_deviceName);
  }

  if (PurgeComm(_prtHdl, PURGE_TXABORT | PURGE_TXCLEAR) == 0)
  {
    throw DeviceWriteException(_deviceName, "PurgeComm failed");
  }

  unsigned long readsz;
  if (WriteFile(_prtHdl, buf, size, &readsz, 0) == 0)
  {
    throw DeviceWriteException(_deviceName, "WriteFile failed");
  }

  if (readsz != static_cast< long >(size))
  {
    throw WriteNotCompleteException(_deviceName);
  }

  return (int)readsz;
}

int CCdlCOM::recv(void* buf, int size)
{
  if (_prtHdl == INVALID_HANDLE_VALUE)
  {
    throw PortNotOpenException(_deviceName);
  }

  unsigned char* tmp = static_cast< unsigned char* >(buf);
  unsigned long readsz = 0, readsz_temp = 0;
  KNI::Timer timeout(_ccd.rttc);
  timeout.Start();
  while (readsz < (unsigned long)size && !timeout.Elapsed())
  {
    if (ReadFile(_prtHdl, &tmp[readsz], size, &readsz_temp, 0) == 0)
    {
      DeviceReadException(_deviceName, "ReadFile failed");
    }
    else
    {
      readsz += readsz_temp;
    }
  }

  if ((unsigned)size != readsz)
  {
    throw ReadNotCompleteException(_deviceName);
  }

  if (PurgeComm(_prtHdl, PURGE_RXABORT | PURGE_RXCLEAR) == 0)
  {
    throw DeviceReadException(_deviceName, "PurgeComm failed");
  }
  return (int)readsz;
}

#else  // LINUX

CCdlCOM::CCdlCOM(TCdlCOMDesc ccd) : _deviceName(""), _ccd(), _prtHdl(-1), _oto()
{
  int prtHdl = -1;

  std::string deviceName;
  struct termios nto, oto;
  char name[11];

  errno = 0;

  strncpy(name, "/dev/ttyS ", 11);
  name[9] = digit(ccd.port);
  prtHdl = ::open(name, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

  _deviceName = name;

  if (prtHdl < 0)
  {
    throw CannotOpenPortException(_deviceName, strerror(errno));
  }

  tcgetattr(prtHdl, &oto);
  bzero(&nto, sizeof(nto));
  nto.c_cc[VTIME] = 0;
  nto.c_cc[VMIN] = 0;
  nto.c_oflag = 0;
  nto.c_lflag = 0;
  nto.c_cflag = CLOCAL | CREAD;
  nto.c_iflag = 0;

  switch (ccd.baud)
  {
    case 50:
      nto.c_cflag |= B50;
      break;
    case 75:
      nto.c_cflag |= B75;
      break;
    case 110:
      nto.c_cflag |= B110;
      break;
    case 134:
      nto.c_cflag |= B134;
      break;
    case 150:
      nto.c_cflag |= B150;
      break;
    case 200:
      nto.c_cflag |= B200;
      break;
    case 300:
      nto.c_cflag |= B300;
      break;
    case 600:
      nto.c_cflag |= B600;
      break;
    case 1200:
      nto.c_cflag |= B1200;
      break;
    case 1800:
      nto.c_cflag |= B1800;
      break;
    case 2400:
      nto.c_cflag |= B2400;
      break;
    case 4800:
      nto.c_cflag |= B4800;
      break;
    case 9600:
      nto.c_cflag |= B9600;
      break;
    case 19200:
      nto.c_cflag |= B19200;
      break;
    case 38400:
      nto.c_cflag |= B38400;
      break;
    case 57600:
      nto.c_cflag |= B57600;
      break;
    case 115200:
      nto.c_cflag |= B115200;
      break;
    case 230400:
      nto.c_cflag |= B230400;
      break;
  }

  switch (ccd.data)
  {
    case 5:
      nto.c_cflag |= CS5;
      break;
    case 6:
      nto.c_cflag |= CS6;
      break;
    case 7:
      nto.c_cflag |= CS7;
      break;
    case 8:
      nto.c_cflag |= CS8;
      break;
  }

  switch (ccd.parity)
  {
    case 'N':
    case 'n':
      break;
    case 'O':
    case 'o':
      nto.c_cflag |= PARENB | PARODD;
      break;
    case 'E':
    case 'e':
      nto.c_cflag |= PARENB;
      break;
  }

  switch (ccd.stop)
  {
    case 1:
      break;
    case 2:
      nto.c_cflag |= CSTOPB;
      break;
  }

  tcflush(prtHdl, TCIFLUSH);
  if (tcsetattr(prtHdl, TCSANOW, &nto) != 0)
  {
    ::close(prtHdl);
    throw CannotGetSetPortAttributesException(_deviceName);
  }

  _prtHdl = prtHdl;
  _deviceName = deviceName;
  _ccd = ccd;
  _oto = oto;
}

CCdlCOM::~CCdlCOM()
{
  if (_prtHdl < 0)
  {
    return;
  }

  tcflush(_prtHdl, TCIFLUSH);
  tcsetattr(_prtHdl, TCSANOW, &_oto);

  ::close(_prtHdl);  // there's nothing we can do about failing
}

int CCdlCOM::send(const void* buf, int size)
{
  if (_prtHdl < 0)
    throw PortNotOpenException(_deviceName);

  errno = 0;

  int tcflush_return = tcflush(_prtHdl, TCIFLUSH);
  if (tcflush_return < 0)
    throw DeviceWriteException(_deviceName, strerror(errno));

  int writesz = write(_prtHdl, buf, size);

  if (writesz < 0)
    throw DeviceWriteException(_deviceName, strerror(errno));

  if (writesz != size)
    throw WriteNotCompleteException(_deviceName);

  return writesz;
}

int CCdlCOM::recv(void* buf, int size)
{
  unsigned char* tmp = static_cast< unsigned char* >(buf);
  register int readsz = 0;

  if (_prtHdl < 0)
    throw PortNotOpenException(_deviceName);

  errno = 0;

  int read_return;
  KNI::Timer timeout(_ccd.rttc);
  timeout.Start();
  while (readsz < size && !timeout.Elapsed())
  {
    read_return = read(_prtHdl, &tmp[readsz], size - readsz);
    if (read_return < 0)
    {
      if (errno != EAGAIN)
        throw DeviceReadException(_deviceName, strerror(errno));
    }
    else
    {
      readsz += read_return;
    }
  }

  if (readsz != size)
    throw ReadNotCompleteException(_deviceName);

  int tcflush_return = tcflush(_prtHdl, TCIFLUSH);
  if (tcflush_return < 0)
    throw DeviceReadException(_deviceName, strerror(errno));

  return readsz;
}

#endif  // WIN32 else LINUX
