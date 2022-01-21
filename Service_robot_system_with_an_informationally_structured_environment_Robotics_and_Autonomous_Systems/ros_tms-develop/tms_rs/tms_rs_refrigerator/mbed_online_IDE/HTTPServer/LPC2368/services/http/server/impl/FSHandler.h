
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

#ifndef FS_HANDLER_H
#define FS_HANDLER_H

#include "../HTTPRequestHandler.h"
#include "mbed.h"

#include <map>
using std::map;

#include <string>
using std::string;

class FSHandler : public HTTPRequestHandler
{
public:
  FSHandler(const char* rootPath, const char* path, TCPSocket* pTCPSocket);
  virtual ~FSHandler();

  static void mount(const string& fsPath, const string& rootPath);

  // protected:
  static inline HTTPRequestHandler* inst(const char* rootPath, const char* path, TCPSocket* pTCPSocket)
  {
    return new FSHandler(rootPath, path, pTCPSocket);
  }  // if we ever could do static virtual functions, this would be one

  virtual void doGet();
  virtual void doPost();
  virtual void doHead();

  virtual void onReadable();   // Data has been read
  virtual void onWriteable();  // Data has been written & buf is free
  virtual void onClose();      // Connection is closing

private:
  FILE* m_fp;
  bool m_err404;
  static map< string, string > m_lFsPath;
};

#endif
