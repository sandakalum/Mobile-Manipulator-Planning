
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

#ifndef NETIF_H
#define NETIF_H

#include "core/ipaddr.h"
/*
#include "nettcpsocket.h"
#include "netudpsocket.h"
#include "netdnsrequest.h"
*/
class NetTcpSocket;
class NetUdpSocket;
class NetDnsRequest;

#if 0
enum NetifEvent
{
  NETIF_CONNECTED, //Connected, can create & use sockets now
  NETIF_DNSREPLY,
  NETIF_DISCONNECTED
};
#endif

class NetIf
{
public:
  NetIf();
  virtual ~NetIf();
  virtual NetTcpSocket* tcpSocket() = 0;  // Create a new tcp socket
  virtual NetUdpSocket* udpSocket() = 0;  // Create a new udp socket
  virtual void poll() = 0;
  virtual NetDnsRequest* dnsRequest(const char* hostname) = 0;  // Create a new NetDnsRequest object
  virtual NetDnsRequest* dnsRequest(Host* pHost) = 0;           // Create a new NetDnsRequest object

  //!Returns the IP of the interface once it's connected
  IpAddr getIp() const;

protected:
  IpAddr m_ip;
};

#endif
