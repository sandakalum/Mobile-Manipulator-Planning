
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
#include "KNI/cdlSocket.h"
#include <iostream>
/****************************************************************************/
// test write:
int writesz;
//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//
// Default arguments for local Katana Simulator:
CCdlSocket::CCdlSocket(char* ipAddr, int port) : _ipAddr(ipAddr), _port(port)
{
  // Getting the version and correct Winsock dll:
  WORD wVersionRequested = MAKEWORD(2, 2);
  WSADATA wdData;
  WSAStartup(wVersionRequested, &wdData);
  _socketfd = INVALID_SOCKET;
  if ((_socketfd = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
  {
    return;
  }
  _socketAddr.sin_family = AF_INET;
  _socketAddr.sin_addr.s_addr = inet_addr(_ipAddr);
  _socketAddr.sin_port = htons(_port);
  if (connect(_socketfd, (SOCKADDR*)&_socketAddr, sizeof(_socketAddr)) == SOCKET_ERROR)
  {
    std::cout << "client could not connect, check if server is running\n";
    return;
  }
}

CCdlSocket::~CCdlSocket()
{
  // close the socket:
  closesocket(_socketfd);
  WSACleanup();
}

int CCdlSocket::send(const void* _buf, int _size)
{
  writesz = -1;
  writesz = ::send(_socketfd, (char*)_buf, _size, 0);
  if (writesz < 0)
    throw DeviceWriteException(_ipAddr, strerror(errno));
  if (writesz != _size)
  {
    throw WriteNotCompleteException(_ipAddr);
  }
  return writesz;
}

int CCdlSocket::recv(void* _buf, int _size)
{
  int read_return;
  // recv is blocking...
  read_return = ::recv(_socketfd, (char*)_buf, _size, 0);
  if (read_return < 0)
  {
    throw DeviceReadException(_ipAddr, strerror(errno));
  }
  // if (read_return != _size) {
  if (read_return == 0)
  {
    throw ReadNotCompleteException(_ipAddr);
  }
  return read_return;
}

int CCdlSocket::disconnect()
{
  closesocket(_socketfd);
  WSACleanup();

  return 0;
}
//-------------------------------------------------------//
#else   // LINUX
//-------------------------------------------------------//
// Default arguments for local Katana Simulator:
CCdlSocket::CCdlSocket(char* ipAddr, int port) : _ipAddr(ipAddr), _port(port)
{
  // filling in the _socketAddr structure:
  int res;
  _socketfd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (_socketfd == -1)
  {
    std::cout << "socket could not be created" << _ipAddr << " port: " << _port << " \n";
    exit(1);
  }
  memset(&_socketAddr, 0, sizeof(_socketAddr));
  _socketAddr.sin_family = AF_INET;
  _socketAddr.sin_addr.s_addr = inet_addr(_ipAddr);
  _socketAddr.sin_port = htons(_port);
  _len = sizeof(_socketAddr);
  // connect to the server
  res = inet_pton(AF_INET, _ipAddr, &_socketAddr.sin_addr);
  if (errno == EAFNOSUPPORT)
  {
    std::cout << "inet_pton failed, try again " << _ipAddr << " port: " << _port << " \n";
    exit(1);
  }
  //	std::cout << "trying to connect to server...\n";
  res = connect(_socketfd, (struct sockaddr*)&_socketAddr, _len);
  if (res != 0)
  {
    std::cout << "client could not connect, check if server is running on ip " << _ipAddr << " port: " << _port << " "
                                                                                                                   "\n";
    exit(1);
  }
  else
  {
    //		std::cout << "client connected to ip "<<_ipAddr<<", port: "<< _port<<" \n";
  }
}

CCdlSocket::~CCdlSocket()
{
  // close the socket:
  close(_socketfd);
}

int CCdlSocket::send(const void* _buf, int _size)
{
  writesz = -1;
  writesz = ::send(_socketfd, _buf, _size, 0 /*MSG_NOSIGNAL*/);
  if (writesz < 0)
  {
    throw DeviceWriteException(_ipAddr, strerror(errno));
  }

  if (writesz != _size)
  {
    throw WriteNotCompleteException(_ipAddr);
    std::cout << "Write not complete: size and length of buf written do not match\n";
  }
  else
  {
    //		std::cout << "written: " << _buf << " _size: " << writesz << std::endl;
  }
  return writesz;
}

int CCdlSocket::recv(void* _buf, int _size)
{
  int read_return = 0;
  read_return = read(_socketfd, _buf, _size);
  // if (read_return != _size) {
  if (read_return <= 0)
  {
    throw ReadNotCompleteException(_ipAddr);
    std::cout << "Read not complete. Buffer size and message length do not match. buf: " << _buf
              << " _size: " << read_return << std::endl;
  }
  else
  {
    //		std::cout << "received: " << _buf << " _size: " << read_return << std::endl;
  }
  return read_return;
}

int CCdlSocket::disconnect()
{
  // close the socket:
  close(_socketfd);
  return 0;
}

//-------------------------------------------------------//
#endif  // WIN32 else LINUX
//-------------------------------------------------------//
