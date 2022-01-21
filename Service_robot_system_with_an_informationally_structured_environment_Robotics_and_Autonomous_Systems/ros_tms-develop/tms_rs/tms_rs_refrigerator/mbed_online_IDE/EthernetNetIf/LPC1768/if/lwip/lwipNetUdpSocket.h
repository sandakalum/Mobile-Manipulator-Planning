
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

#ifndef LWIPNETUDPSOCKET_H
#define LWIPNETUDPSOCKET_H

#define NET_LWIP_STACK 1
//#include "lwip/ip_addr.h"
#include "if/net/netudpsocket.h"
#include "LwipNetIf.h"

#include "stdint.h"

#include <list>
using std::list;

// Implements NetUdpSockets over lwIP raw API

struct udp_pcb;  // Represents a Udp Connection, "Protocol Control Block", see rawapi.txt & udp.h
struct pbuf;     // Lwip Buffer Container
typedef struct ip_addr ip_addr_t;

// typedef signed char err_t;
typedef uint16_t u16_t;

class LwipNetUdpSocket : public NetUdpSocket
{
public:
  LwipNetUdpSocket(udp_pcb* pPcb = NULL);  // Passes a pcb if already created (by an accept req for instance), in that
                                           // case transfers ownership
  virtual ~LwipNetUdpSocket();

  virtual NetUdpSocketErr bind(const Host& me);

  virtual int /*if < 0 : NetUdpSocketErr*/ sendto(const char* buf, int len, Host* pHost);
  virtual int /*if < 0 : NetUdpSocketErr*/ recvfrom(char* buf, int len, Host* pHost);

  virtual NetUdpSocketErr close();

  virtual NetUdpSocketErr poll();

protected:
  volatile udp_pcb* m_pPcb;

  // Event callback from lwIp
  void recvCb(udp_pcb* pcb, struct pbuf* p, ip_addr_t* addr, u16_t port);

private:
  void cleanUp();  // Flush input buffer
  struct InPacket
  {
    volatile pbuf* pBuf;
    ip_addr_t addr;
    u16_t port;
  };

  list< InPacket > m_lInPkt;
  IpAddr m_multicastGroup;

  // Static callback : Transforms into a C++ callback
  static void sRecvCb(void* arg, struct udp_pcb* pcb, struct pbuf* p, ip_addr_t* addr, u16_t port);
};

#endif
