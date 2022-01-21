
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

#ifndef BASE64_H
#define BASE64_H

#include <string>
using std::string;

#ifdef __cplusplus
extern "C" {
#endif

// Originaly from Rolf's iputil.h

unsigned int base64enc_len(const char *str);

void base64enc(const char *input, unsigned int length, char *output);

#ifdef __cplusplus
}
#endif

class Base64
{
public:
  static string encode(const string &str)
  {
    char *out = new char[base64enc_len(str.c_str())];
    base64enc(str.c_str(), str.length(), out);
    string res(out);
    delete[] out;
    return res;
  }
};

#endif /* LWIP_UTILS_H */
