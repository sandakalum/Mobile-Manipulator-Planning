/*
Copyright (c) 2010 ARM Ltd

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
#include "mbed.h"
#include "EthernetNetIf.h"
#include "HTTPServer.h"

DigitalOut led1(LED1, "led1");
DigitalOut led2(LED2, "led2");
DigitalOut led3(LED3, "led3");
DigitalOut led4(LED4, "led4");

LocalFileSystem fs("webfs");

EthernetNetIf eth(IpAddr(192, 168, 4, 239),  // IP Address
                  IpAddr(255, 255, 255, 0),  // Network Mask
                  IpAddr(192, 168, 4, 140),  // Gateway
                  IpAddr(133, 5, 6, 1)       // DNS
                  );

HTTPServer svr;

int main()
{
  Base::add_rpc_class< AnalogIn >();
  Base::add_rpc_class< AnalogOut >();
  Base::add_rpc_class< DigitalIn >();
  Base::add_rpc_class< DigitalOut >();
  Base::add_rpc_class< DigitalInOut >();
  Base::add_rpc_class< PwmOut >();
  Base::add_rpc_class< Timer >();
  Base::add_rpc_class< BusOut >();
  Base::add_rpc_class< BusIn >();
  Base::add_rpc_class< BusInOut >();
  Base::add_rpc_class< Serial >();

  printf("Setting up...\n");
  EthernetErr ethErr = eth.setup();

  if (ethErr)
  {
    printf("Error %d in setup.\n", ethErr);
    return -1;
  }
  printf("Setup OK\n");

  FSHandler::mount("/webfs", "/files");  // Mount /webfs path on /files web path
  FSHandler::mount("/webfs", "/");       // Mount /webfs path on web root path

  svr.addHandler< SimpleHandler >("/hello");
  svr.addHandler< RPCHandler >("/rpc");
  svr.addHandler< FSHandler >("/files");
  svr.addHandler< FSHandler >("/");  // Default handler
  // Example : Access to mbed.htm : http://a.b.c.d/mbed.htm or http://a.b.c.d/files/mbed.htm

  svr.bind(80);

  printf("Listening...\n");

  Timer tm;
  tm.start();
  // Listen indefinitely
  while (true)
  {
    Net::poll();
    if (tm.read() > .5)
    {
      led1 = !led1;  // Show that we are alive
      tm.start();
    }
  }

  return 0;
}
