
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

#ifndef LWIPNETTCPSOCKET_H
#define LWIPNETTCPSOCKET_H

#define NET_LWIP_STACK 1
#include "if/net/nettcpsocket.h"
#include "LwipNetIf.h"

#include "stdint.h"

// Implements NetTcpSockets over lwIP raw API

struct tcp_pcb;  // Represents a Tcp Connection, "Protocol Control Block", see rawapi.txt & tcp.h
struct pbuf;     // Lwip Buffer Container

typedef signed char err_t;
typedef uint16_t u16_t;

class LwipNetTcpSocket : public NetTcpSocket
{
public:
  LwipNetTcpSocket(tcp_pcb* pPcb = NULL);  // Passes a pcb if already created (by an accept req for instance), in that
                                           // case transfers ownership
  virtual ~LwipNetTcpSocket();

  virtual NetTcpSocketErr bind(const Host& me);
  virtual NetTcpSocketErr listen();
  virtual NetTcpSocketErr connect(const Host& host);
  virtual NetTcpSocketErr accept(Host* pClient, NetTcpSocket** ppNewNetTcpSocket);

  virtual int /*if < 0 : NetTcpSocketErr*/ send(const char* buf, int len);
  virtual int /*if < 0 : NetTcpSocketErr*/ recv(char* buf, int len);

  virtual NetTcpSocketErr close();

  virtual NetTcpSocketErr poll();

protected:
  volatile tcp_pcb* m_pPcb;

  // Events callbacks from lwIp
  err_t acceptCb(tcp_pcb* newpcb, err_t err);
  err_t connectedCb(tcp_pcb* tpcb, err_t err);

  void errCb(err_t err);

  err_t sentCb(tcp_pcb* tpcb, u16_t len);
  err_t recvCb(tcp_pcb* tpcb, pbuf* p, err_t err);

private:
  void cleanUp();  // Flush input buffer

  //  queue<tcp_pcb*> m_lpInPcb; //Incoming connections that have not been accepted yet
  queue< LwipNetTcpSocket* > m_lpInNetTcpSocket;  // Incoming connections that have not been accepted yet

  volatile pbuf* m_pReadPbuf;  // Ptr to read buffer

  // Static callbacks : Transforms into a C++ callback
  static err_t sAcceptCb(void* arg, struct tcp_pcb* newpcb, err_t err);
  static err_t sConnectedCb(void* arg, struct tcp_pcb* tpcb, err_t err);

  static void sErrCb(void* arg, err_t err);

  static err_t sSentCb(void* arg, struct tcp_pcb* tpcb, u16_t len);
  static err_t sRecvCb(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);
};

#endif
