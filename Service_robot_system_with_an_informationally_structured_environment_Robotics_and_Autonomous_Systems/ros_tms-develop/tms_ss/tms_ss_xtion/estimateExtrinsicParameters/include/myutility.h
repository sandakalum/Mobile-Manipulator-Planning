#ifndef _MY_UTILITY_H_
#define _MY_UTILITY_H_

#include <opencv2/opencv.hpp>

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#endif

#define KEYCODE_d 0x64
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72

class KeyboardEventReader
{
public:
  KeyboardEventReader();
  ~KeyboardEventReader();
  bool getKeycode(char &c);

private:
  int kfd;
  bool dirty;
  struct termios cooked;
  struct termios raw;
};

#endif
