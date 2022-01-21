#include <iostream>
#include <vector>
//#include <OpenNI.h>

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#endif

#include "myutility.h"

/**--------------------------------------------------
 * \class KeyboardEventReader
 */
KeyboardEventReader::KeyboardEventReader()
{
  kfd = 0;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);

  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

KeyboardEventReader::~KeyboardEventReader()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

bool KeyboardEventReader::getKeycode(char &c)
{
  if (read(kfd, &c, 1) < 0)
  {
    perror("read():");
    return false;
  }
  return true;
}
/*---------------------------------------------------*/
