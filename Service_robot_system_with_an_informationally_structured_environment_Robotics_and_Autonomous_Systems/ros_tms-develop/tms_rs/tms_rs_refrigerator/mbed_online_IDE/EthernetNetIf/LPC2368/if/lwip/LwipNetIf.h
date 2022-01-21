
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

#ifndef LWIPNETIF_H
#define LWIPNETIF_H

// class Ticker;
#include "mbed.h"

#define NET_LWIP_STACK 1
#include "core/net.h"
#include "if/net/netif.h"
/*
#include "lwipNetTcpSocket.h"
#include "lwipNetUdpSocket.h"
#include "lwipNetDnsRequest.h"
*/

class LwipNetIf : public NetIf
{
public:
  LwipNetIf();
  virtual ~LwipNetIf();

  void init();

  virtual NetTcpSocket* tcpSocket();                        // Create a new tcp socket
  virtual NetUdpSocket* udpSocket();                        // Create a new udp socket
  virtual NetDnsRequest* dnsRequest(const char* hostname);  // Create a new NetDnsRequest object
  virtual NetDnsRequest* dnsRequest(Host* pHost);           // Create a new NetDnsRequest object

  virtual void poll();

private:
  Timer m_tcpTimer;
  Timer m_dnsTimer;

  bool m_init;
};

#endif
