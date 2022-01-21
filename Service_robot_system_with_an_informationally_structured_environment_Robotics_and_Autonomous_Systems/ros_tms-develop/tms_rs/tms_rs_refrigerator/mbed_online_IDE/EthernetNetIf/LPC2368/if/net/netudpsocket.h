
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

#ifndef NETUDPSOCKET_H
#define NETUDPSOCKET_H

#include "net.h"
#include "host.h"

#include <queue>
using std::queue;

// Implements a Berkeley-like socket if
// Can be interfaced either to lwip or a Telit module

enum NetUdpSocketErr
{
  __NETUDPSOCKET_MIN = -0xFFFF,
  NETUDPSOCKET_SETUP,  // NetUdpSocket not properly configured
  NETUDPSOCKET_IF,     // If has problems
  NETUDPSOCKET_MEM,    // Not enough mem
  NETUDPSOCKET_INUSE,  // If/Port is in use
  //...
  NETUDPSOCKET_OK = 0
};

enum NetUdpSocketEvent  // Only one lonely event here... but who knows, maybe some day there'll be another one!
{
  NETUDPSOCKET_READABLE,  // Data in buf
};

class NetUdpSocket
{
public:
  NetUdpSocket();
  virtual ~NetUdpSocket();  // close()

  virtual NetUdpSocketErr bind(const Host& me) = 0;

  virtual int /*if < 0 : NetUdpSocketErr*/ sendto(const char* buf, int len, Host* pHost) = 0;
  virtual int /*if < 0 : NetUdpSocketErr*/ recvfrom(char* buf, int len, Host* pHost) = 0;

  /* TODO NTH : printf / scanf helpers that call send/recv */

  virtual NetUdpSocketErr close() = 0;

  virtual NetUdpSocketErr poll() = 0;

  class CDummy;
  // Callbacks
  template < class T >
  // Linker bug : Must be defined here :(
  void setOnEvent(T* pItem, void (T::*pMethod)(NetUdpSocketEvent))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(NetUdpSocketEvent))pMethod;
  }

  void resetOnEvent();  // Disable callback

protected:
  void queueEvent(NetUdpSocketEvent e);
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
  void onEvent(NetUdpSocketEvent e);  // To be called on poll
  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(NetUdpSocketEvent);
  queue< NetUdpSocketEvent > m_events;
};

#endif
