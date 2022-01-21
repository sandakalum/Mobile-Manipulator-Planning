
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

#ifndef HTTP_REQUEST_DISPATCHER_H
#define HTTP_REQUEST_DISPATCHER_H

class HTTPServer;

#include "api/TCPSocket.h"
#include "HTTPServer.h"
#include "core/netservice.h"

#include "mbed.h"

#define HTTP_REQUEST_TIMEOUT 5000

#include <string>
using std::string;

class HTTPRequestDispatcher : public NetService
{
public:
  HTTPRequestDispatcher(HTTPServer* pSvr, TCPSocket* pTCPSocket);
  virtual ~HTTPRequestDispatcher();

private:
  enum HTTP_METH
  {
    HTTP_GET,
    HTTP_POST,
    HTTP_HEAD
  };

  void dispatchRequest();

  virtual void close();  // Close TCPSocket and destroy data

  void onTCPSocketEvent(TCPSocketEvent e);

  void onTimeout();  // Connection has timed out

  bool getRequest(string* path, string* meth);

  HTTPServer* m_pSvr;
  TCPSocket* m_pTCPSocket;

  Timeout m_watchdog;
  bool m_closed;
};

#endif
