
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

#ifndef NETTCPSOCKET_H
#define NETTCPSOCKET_H

#include "net.h"
#include "host.h"

#include <queue>
using std::queue;

// Implements a Berkeley-like socket if
// Can be interfaced either to lwip or a Telit module

enum NetTcpSocketErr
{
  __NETTCPSOCKET_MIN = -0xFFFF,
  NETTCPSOCKET_SETUP,  // NetTcpSocket not properly configured
  NETTCPSOCKET_TIMEOUT,
  NETTCPSOCKET_IF,     // If has problems
  NETTCPSOCKET_MEM,    // Not enough mem
  NETTCPSOCKET_INUSE,  // If/Port is in use
  NETTCPSOCKET_EMPTY,  // Connections queue is empty
  NETTCPSOCKET_RST,    // Connection was reset by remote host
                       //...
  NETTCPSOCKET_OK = 0
};

enum NetTcpSocketEvent
{
  NETTCPSOCKET_CONNECTED,  // Connected to host, must call accept() if we were listening
  NETTCPSOCKET_ACCEPT,     // Connected to client
  NETTCPSOCKET_READABLE,   // Data in buf
  NETTCPSOCKET_WRITEABLE,  // Can write data to buf
  NETTCPSOCKET_CONTIMEOUT,
  NETTCPSOCKET_CONRST,
  NETTCPSOCKET_CONABRT,
  NETTCPSOCKET_ERROR,
  NETTCPSOCKET_DISCONNECTED
};

class NetTcpSocket
{
public:
  NetTcpSocket();
  virtual ~NetTcpSocket();  // close()

  virtual NetTcpSocketErr bind(const Host& me) = 0;
  virtual NetTcpSocketErr listen() = 0;
  virtual NetTcpSocketErr connect(const Host& host) = 0;
  virtual NetTcpSocketErr accept(Host* pClient, NetTcpSocket** ppNewNetTcpSocket) = 0;

  virtual int /*if < 0 : NetTcpSocketErr*/ send(const char* buf, int len) = 0;
  virtual int /*if < 0 : NetTcpSocketErr*/ recv(char* buf, int len) = 0;

  virtual NetTcpSocketErr close() = 0;

  virtual NetTcpSocketErr poll() = 0;

  class CDummy;
  // Callbacks
  template < class T >
  // Linker bug : Must be defined here :(
  void setOnEvent(T* pItem, void (T::*pMethod)(NetTcpSocketEvent))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(NetTcpSocketEvent))pMethod;
  }

  void resetOnEvent();  // Disable callback

protected:
  void queueEvent(NetTcpSocketEvent e);
  void discardEvents();
  void flushEvents();  // to be called during polling

  Host m_host;
  Host m_client;

  friend class Net;
  int m_refs;

  bool m_closed;
  bool m_removed;

private:
  // We do not want to execute user code in interrupt routines, so we queue events until the server is polled
  // If we port this to a multithreaded OS, we could avoid this (however some functions here are not thread-safe, so
  // beware ;) )
  void onEvent(NetTcpSocketEvent e);  // To be called on poll
  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(NetTcpSocketEvent);
  queue< NetTcpSocketEvent > m_events;
};

#endif
