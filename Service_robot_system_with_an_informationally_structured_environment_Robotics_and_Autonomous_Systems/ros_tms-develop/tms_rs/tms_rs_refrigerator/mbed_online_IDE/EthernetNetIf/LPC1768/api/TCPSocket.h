
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
TCP Socket header file
*/

#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include "core/net.h"
#include "core/host.h"
// Essentially it is a safe interface to NetTcpSocket

/// TCP Socket error codes
enum TCPSocketErr
{
  __TCPSOCKET_MIN = -0xFFFF,
  TCPSOCKET_SETUP,    ///<TCPSocket not properly configured
  TCPSOCKET_TIMEOUT,  ///<Connection timed out
  TCPSOCKET_IF,       ///<Interface has problems, does not exist or is not initialized
  TCPSOCKET_MEM,      ///<Not enough mem
  TCPSOCKET_INUSE,    ///<Interface / Port is in use
  TCPSOCKET_EMPTY,    ///<Connections queue is empty
  TCPSOCKET_RST,      ///<Connection was reset by remote host
                      //...
  TCPSOCKET_OK = 0    ///<Success
};

/// TCP Socket Events
enum TCPSocketEvent
{
  TCPSOCKET_CONNECTED,    ///<Connected to host
  TCPSOCKET_ACCEPT,       ///<Client is connected, must call accept() to get a new Socket
  TCPSOCKET_READABLE,     ///<Data in buf
  TCPSOCKET_WRITEABLE,    ///<Can write data to buf
  TCPSOCKET_CONTIMEOUT,   ///<Connection timed out
  TCPSOCKET_CONRST,       ///<Connection was reset by remote host
  TCPSOCKET_CONABRT,      ///<Connection was aborted
  TCPSOCKET_ERROR,        ///<Unknown error
  TCPSOCKET_DISCONNECTED  ///<Disconnected
};

class NetTcpSocket;
enum NetTcpSocketEvent;

/// This is a simple TCP Socket class
/**
  This class exposes an API to deal with TCP Sockets
*/
class TCPSocket
{
public:
  /// Creates a new socket
  TCPSocket();

protected:
  TCPSocket(NetTcpSocket* pNetTcpSocket);

public:
  /// Closes if needed and destroys the socket
  ~TCPSocket();  // close()

  /// Binds the socket to (local) host
  TCPSocketErr bind(const Host& me);

  /// Starts listening
  TCPSocketErr listen();

  /// Connects socket to host
  TCPSocketErr connect(const Host& host);

  /// Accepts connection from client and gets connected socket
  TCPSocketErr accept(Host* pClient, TCPSocket** ppNewTcpSocket);

  /// Sends data
  /*
  @return a negative error code or the number of bytes transmitted
  */
  int /*if < 0 : TCPSocketErr*/ send(const char* buf, int len);

  /// Receives data
  /*
  @return a negative error code or the number of bytes received
  */
  int /*if < 0 : TCPSocketErr*/ recv(char* buf, int len);

  /* TODO NTH : printf / scanf helpers that call send/recv */

  /// Closes socket
  TCPSocketErr close();

  // Callbacks
  /// Setups callback
  /**
  @param pMethod : callback function
  */
  void setOnEvent(void (*pMethod)(TCPSocketEvent));

  class CDummy;
  /// Setups callback
  /**
  @param pItem : instance of class on which to execute the callback method
  @param pMethod : callback method
  */
  template < class T >
  void setOnEvent(T* pItem, void (T::*pMethod)(TCPSocketEvent))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(TCPSocketEvent))pMethod;
  }

  /// Disables callback
  void resetOnEvent();

protected:
  void onNetTcpSocketEvent(NetTcpSocketEvent e);
  TCPSocketErr checkInst();

private:
  NetTcpSocket* m_pNetTcpSocket;

  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(TCPSocketEvent);

  void (*m_pCb)(TCPSocketEvent);
};

#endif
