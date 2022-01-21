
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

#ifndef NETDNSREQUEST_H
#define NETDNSREQUEST_H

// class Socket;
class Host;
// class NetService;
class NetDnsRequest;

#include "netservice.h"
#include "ipaddr.h"
#include "host.h"

enum NetDnsReply
{
  NETDNS_PRTCL,
  NETDNS_NOTFOUND,  // Hostname is unknown
  NETDNS_ERROR,     // Problem with DNS Service
  //...
  NETDNS_FOUND,
};

class NetDnsRequest : public NetService
{
public:
  NetDnsRequest(const char* hostname);
  NetDnsRequest(Host* pHost);
  virtual ~NetDnsRequest();

  class CDummy;
  template < class T >
  // Linker bug : Must be defined here :(
  void setOnReply(T* pItem, void (T::*pMethod)(NetDnsReply))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(NetDnsReply))pMethod;
  }

  // Execute request & return OK if found, NOTFOUND or ERROR on error, or PROCESSING if the request has not completed
  // yet
  // virtual DnsErr pollState() = 0;
  virtual void poll() = 0;  // NetService fn

  void getResult(IpAddr* pIp);

  virtual void close();

protected:
  void onReply(NetDnsReply reply);  // Must be called by impl when the request completes

  IpAddr m_ip;
  char* m_hostname;

private:
  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(NetDnsReply);

  Host* m_pHost;

  bool m_closed;
};

#endif
