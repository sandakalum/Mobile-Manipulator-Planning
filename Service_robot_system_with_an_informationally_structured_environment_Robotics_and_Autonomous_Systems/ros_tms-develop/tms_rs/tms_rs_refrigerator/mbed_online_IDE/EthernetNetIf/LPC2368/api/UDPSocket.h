
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
UDP Socket header file
*/

#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include "core/net.h"
#include "core/host.h"
// Essentially it is a safe interface to NetUdpSocket

/// UDP Socket error codes
enum UDPSocketErr
{
  __UDPSOCKET_MIN = -0xFFFF,
  UDPSOCKET_SETUP,  ///<UDPSocket not properly configured
  UDPSOCKET_IF,     ///<Interface has problems, does not exist or is not initialized
  UDPSOCKET_MEM,    ///<Not enough mem
  UDPSOCKET_INUSE,  ///<Interface / Port is in use
                    //...
  UDPSOCKET_OK = 0  ///<Success
};

/// UDP Socket Event(s)
enum UDPSocketEvent  // Only one event here for now, but keeps that model in case we need to implement some others
{
  UDPSOCKET_READABLE,  ///<Data in buf
};

class NetUdpSocket;
enum NetUdpSocketEvent;

/// This is a simple UDP Socket class
/**
  This class exposes an API to deal with UDP Sockets
*/
class UDPSocket
{
public:
  /// Creates a new socket
  UDPSocket();

  /// Closes and destroys socket
  ~UDPSocket();  // close()

  /// Binds the socket to local host or a multicast address
  UDPSocketErr bind(const Host& me);

  /// Sends data
  /*
  @param pHost : host to send data to
  @return a negative error code or the number of bytes transmitted
  */
  int /*if < 0 : UDPSocketErr*/ sendto(const char* buf, int len, Host* pHost);

  /// Receives data
  /*
  @param pHost : host from which this piece of data comes from
  @return a negative error code or the number of bytes received
  */
  int /*if < 0 : UDPSocketErr*/ recvfrom(char* buf, int len, Host* pHost);

  /* TODO NTH : printf / scanf helpers that call send/recv */

  /// Closes socket
  UDPSocketErr close();

  // Callbacks
  /// Setups callback
  /**
  @param pMethod : callback function
  */
  void setOnEvent(void (*pMethod)(UDPSocketEvent));

  class CDummy;
  /// Setups callback
  /**
  @param pItem : instance of class on which to execute the callback method
  @param pMethod : callback method
  */
  template < class T >
  void setOnEvent(T* pItem, void (T::*pMethod)(UDPSocketEvent))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(UDPSocketEvent))pMethod;
  }

  /// Disables callback
  void resetOnEvent();

protected:
  void onNetUdpSocketEvent(NetUdpSocketEvent e);
  UDPSocketErr checkInst();

private:
  NetUdpSocket* m_pNetUdpSocket;

  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(UDPSocketEvent);

  void (*m_pCb)(UDPSocketEvent);
};

#endif
