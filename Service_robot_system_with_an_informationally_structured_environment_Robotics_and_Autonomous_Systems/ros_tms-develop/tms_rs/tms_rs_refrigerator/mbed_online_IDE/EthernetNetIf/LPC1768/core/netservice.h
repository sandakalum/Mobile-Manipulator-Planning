
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
\file Net Service base class header file
*/

#ifndef NETSERVICE_H
#define NETSERVICE_H

#include <list>
using std::list;

/// Net Service base class
/**
Each connection-oriented object can register as service (by inheriting this class), so that it is polled regularly.
It notifies the pool when the connection is terminated so that it can be destroyed.
*/
class NetService
{
public:
  /// Instantiates a new service
  /**
  @param owned If true the object is owned by the pool and will be destroyed on closure.
  */
  NetService(bool owned = true);  // Is owned by the pool?
  virtual ~NetService();

  /// This method can be inherited so that it is called on each @a Net::poll() call.
  virtual void poll();

  static void servicesPoll();  // Poll all registered services & destroy closed ones

protected:
  /// This flags the service as to be destructed if owned by the pool.
  void close();

private:
  bool m_closed;
  bool m_removed;
  bool m_owned;

  static list< NetService* >& lpServices();  // Helper to prevent static initialization fiasco
};

#endif
