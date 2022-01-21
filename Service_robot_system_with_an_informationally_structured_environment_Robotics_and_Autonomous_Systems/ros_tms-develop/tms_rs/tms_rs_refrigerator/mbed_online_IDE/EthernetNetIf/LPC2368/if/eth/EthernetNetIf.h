
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
Ethernet network interface header file
*/

#ifndef ETHERNETNETIF_H
#define ETHERNETNETIF_H

struct netif;

#include "mbed.h"

#include "if/lwip/LwipNetIf.h"

/// Ethernet network interface return codes
enum EthernetErr
{
  __ETH_MIN = -0xFFFF,
  ETH_TIMEOUT,  ///<Timeout during setup
  ETH_OK = 0    ///<Success
};

/// Ethernet network interface
/**
This class provides Ethernet connectivity to the stack
*/
class EthernetNetIf : public LwipNetIf
{
public:
  /// Instantiates the Interface and register it against the stack, DHCP will be used
  EthernetNetIf();  // W/ DHCP

  /// Instantiates the Interface and register it against the stack, DHCP will not be used
  /**
  IpAddr is a container class that can be constructed with either 4 bytes or no parameters for a null IP address.
  */
  EthernetNetIf(IpAddr ip, IpAddr netmask, IpAddr gateway, IpAddr dns);  // W/o DHCP
  virtual ~EthernetNetIf();

  /// Brings the interface up
  /**
  Uses DHCP if necessary
  @param timeout_ms : You can set the timeout parameter in milliseconds, if not it defaults to 15s
  @return : ETH_OK on success or ETH_TIMEOUT on timeout
  */
  EthernetErr setup(int timeout_ms = 15000);

  virtual void poll();

private:
  Timer m_ethArpTimer;
  Timer m_dhcpCoarseTimer;
  Timer m_dhcpFineTimer;
  Timer m_igmpTimer;

  bool m_useDhcp;

  netif* m_pNetIf;

  IpAddr m_netmask;
  IpAddr m_gateway;

  const char* m_hostname;
};

#endif
