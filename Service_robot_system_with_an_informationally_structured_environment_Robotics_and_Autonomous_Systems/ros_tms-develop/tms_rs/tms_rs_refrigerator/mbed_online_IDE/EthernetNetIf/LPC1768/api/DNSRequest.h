
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

/** \file
DNS Request header file
*/

#ifndef DNSREQUEST_H
#define DNSREQUEST_H

#include "core/net.h"
#include "core/ipaddr.h"
#include "core/host.h"
// Essentially it is a safe interface to NetDnsRequest

/// DNS Request error codes
enum DNSRequestErr
{
  __DNS_MIN = -0xFFFF,
  DNS_SETUP,       ///<DNSRequest not properly configured
  DNS_IF,          ///<Interface has problems, does not exist or is not initialized
  DNS_MEM,         ///<Not enough mem
  DNS_INUSE,       ///<Interface / Port is in use
  DNS_PROCESSING,  ///<Request has not completed
                   //...
  DNS_OK = 0       ///<Success
};

/// DNS Request Result Events
enum DNSReply
{
  DNS_PRTCL,
  DNS_NOTFOUND,  /// Hostname is unknown
  DNS_ERROR,     /// Problem with DNS Service
  //...
  DNS_FOUND,
};

class NetDnsRequest;
enum NetDnsReply;

/// This is a simple DNS Request class
/**
  This class exposes an API to deal with DNS Requests
*/
class DNSRequest
{
public:
  /// Creates a new request
  DNSRequest();

  /// Terminates and closes request
  ~DNSRequest();

  /// Resolves an hostname
  /**
  @param hostname : hostname to resolve
  */
  DNSRequestErr resolve(const char* hostname);

  /// Resolves an hostname
  /**
  @param host : hostname to resolve, the result will be stored in the IpAddr field of this object
  */
  DNSRequestErr resolve(Host* pHost);

  /// Setups callback
  /**
  The callback function will be called on result.
  @param pMethod : callback function
  */
  void setOnReply(void (*pMethod)(DNSReply));

  class CDummy;
  /// Setups callback
  /**
  The callback function will be called on result.
  @param pItem : instance of class on which to execute the callback method
  @param pMethod : callback method
  */
  template < class T >
  void setOnReply(T* pItem, void (T::*pMethod)(DNSReply))
  {
    m_pCbItem = (CDummy*)pItem;
    m_pCbMeth = (void (CDummy::*)(DNSReply))pMethod;
  }

  /// Gets IP address once it has been resolved
  /**
  @param pIp : pointer to an IpAddr instance in which to store the resolved IP address
  */
  DNSRequestErr getResult(IpAddr* pIp);

  /// Closes DNS Request before completion
  DNSRequestErr close();

protected:
  void onNetDnsReply(NetDnsReply r);
  DNSRequestErr checkInst();

private:
  NetDnsRequest* m_pNetDnsRequest;

  CDummy* m_pCbItem;
  void (CDummy::*m_pCbMeth)(DNSReply);

  void (*m_pCb)(DNSReply);
};

#endif
