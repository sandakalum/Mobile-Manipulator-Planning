
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

#ifndef NET_H
#define NET_H

class NetIf;
class NetTcpSocket;
class NetUdpSocket;
class NetDnsRequest;

#include <list>
using std::list;

/*
#include "host.h"
#include "ipaddr.h"
#include "netservice.h"
#include "if/net/netif.h"
#include "if/net/nettcpsocket.h"
#include "if/net/netudpsocket.h"
#include "if/net/netdnsrequest.h"
*/

class Host;
class NetIf;
class NetTcpSocket;
class NetUdpSocket;
class NetDnsRequest;

class Net
{
private:
  Net();
  ~Net();

public:
  static void poll();  // Poll every if & socket

  static NetTcpSocket* tcpSocket(NetIf& netif);
  static NetTcpSocket* tcpSocket();  // Socket on default if
  static void releaseTcpSocket(NetTcpSocket* pNetTcpSocket);

  static NetUdpSocket* udpSocket(NetIf& netif);
  static NetUdpSocket* udpSocket();  // Socket on default if
  static void releaseUdpSocket(NetUdpSocket* pNetUdpSocket);

  static NetDnsRequest* dnsRequest(const char* hostname, NetIf& netif);
  static NetDnsRequest* dnsRequest(const char* hostname);  // Create a new NetDnsRequest object from default if

  static NetDnsRequest* dnsRequest(Host* pHost, NetIf& netif);
  static NetDnsRequest* dnsRequest(Host* pHost);  // Create a new NetDnsRequest object from default if

  static void setDefaultIf(NetIf& netif);  // Deprecated
  static void setDefaultIf(NetIf* pIf);
  static NetIf* getDefaultIf();

protected:
  friend class NetIf;
  friend class NetTcpSocket;
  friend class NetUdpSocket;

  static void registerIf(NetIf* pIf);
  static void unregisterIf(NetIf* pIf);

  static void registerNetTcpSocket(NetTcpSocket* pNetTcpSocket);
  static void unregisterNetTcpSocket(NetTcpSocket* pNetTcpSocket);

  static void registerNetUdpSocket(NetUdpSocket* pNetUdpSocket);
  static void unregisterNetUdpSocket(NetUdpSocket* pNetUdpSocket);

private:
  static Net& net();  // Return inst of singleton

  NetIf* m_defaultIf;

  list< NetIf* > m_lpIf;
  list< NetTcpSocket* > m_lpNetTcpSocket;
  list< NetUdpSocket* > m_lpNetUdpSocket;
};

#endif
