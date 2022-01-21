
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

/**
HTTP Request Handler header file.
*/

#ifndef HTTP_REQUEST_HANDLER_H
#define HTTP_REQUEST_HANDLER_H

#include "api/TCPSocket.h"
//#include "HTTPServer.h"

#include "mbed.h"
#include "core/netservice.h"

#include <string>
using std::string;

#include <map>
using std::map;

/// HTTP Server's generic request handler
class HTTPRequestHandler : public NetService
{
public:
  /// Instantiated by the HTTP Server
  HTTPRequestHandler(const char* rootPath, const char* path, TCPSocket* pTCPSocket);
  virtual ~HTTPRequestHandler();

  // protected:
  virtual void doGet() = 0;
  virtual void doPost() = 0;
  virtual void doHead() = 0;

  virtual void onReadable() = 0;   // Data has been read
  virtual void onWriteable() = 0;  // Data has been written & buf is free
  virtual void onTimeout();        // Connection has timed out
  virtual void onClose() = 0;      // Connection is closing

  virtual void close();  // Close socket and destroy data

protected:
  map< string, string >& reqHeaders() /*const*/;
  string& path() /*const*/;
  int dataLen() const;
  int readData(char* buf, int len);
  string& rootPath() /*const*/;

  void setErrCode(int errc);
  void setContentLen(int len);

  map< string, string >& respHeaders();
  int writeData(const char* buf, int len);

  void setTimeout(int ms);
  void resetTimeout();

private:
  void readHeaders();   // Called at instanciation
  void writeHeaders();  // Called at the first writeData call
  void onTCPSocketEvent(TCPSocketEvent e);

  TCPSocket* m_pTCPSocket;
  map< string, string > m_reqHeaders;
  map< string, string > m_respHeaders;
  string m_rootPath;
  string m_path;
  int m_errc;  // Response code

  Timeout m_watchdog;
  int m_timeout;

  bool m_closed;
  bool m_headersSent;

  int readLine(char* str, int maxLen);
};

#endif
