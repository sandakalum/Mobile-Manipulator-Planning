
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

#ifndef URL_H
#define URL_H

#include "core/ipaddr.h"

#include <string>
using std::string;

#include "mbed.h"

#ifdef __cplusplus
extern "C" {
#endif

char* url_encode(char* str);
char* url_decode(char* str);

#ifdef __cplusplus
}
#endif

class Url
{
public:
  static string encode(const string& url)
  {
    char* c_res = url_encode((char*)url.c_str());
    string res(c_res);
    free(c_res);  // Alloc'ed in url_encode()
    return res;
  }

  static string decode(const string& url)
  {
    char* c_res = url_decode((char*)url.c_str());
    string res(c_res);
    free(c_res);  // Alloc'ed in url_decode()
    return res;
  }

  Url();

  string getProtocol();
  string getHost();
  bool getHostIp(IpAddr* ip);  // If host is in IP form, return true & proper object by ptr
  uint16_t getPort();
  string getPath();

  void setProtocol(string protocol);
  void setHost(string host);
  void setPort(uint16_t port);
  void setPath(string path);

  void fromString(string str);
  string toString();

private:
  string m_protocol;
  string m_host;
  uint16_t m_port;
  string m_path;
};

#endif /* LWIP_UTILS_H */
