//------------------------------------------------------------------------------
// @file   : herkulex.cpp
// @brief  : herkulex drive
// @author : Yoonseok Pyo
// @version: Ver0.2 (since 2012.10.01)
// @date   : 2012.10.02
//------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>

#include "herkulex.h"

// make 8bit data
#define LSB(data) ((unsigned char)(data))
#define MSB(data) ((unsigned char)((unsigned int)(data) >> 8) & 0xFF)

typedef struct
{
  unsigned char ucHeader[2];
  unsigned char ucPacketSize;
  unsigned char ucChipID;
  unsigned char ucCmd;
  unsigned char ucCheckSum1;
  unsigned char ucCheckSum2;
  unsigned char ucData[MAX_DATA_SIZE - 2];
} SendIJogPacket;

typedef struct
{
  unsigned char ucHeader[2];
  unsigned char ucPacketSize;
  unsigned char ucChipID;
  unsigned char ucCmd;
  unsigned char ucCheckSum1;
  unsigned char ucCheckSum2;
  unsigned char ucAddress;
  unsigned char ucLen;
  unsigned char ucData[MAX_DATA_SIZE - 2];
} SendRWPacket;

int degToPos(int degree)
{
  return int(3.0695 * degree + 235.5);
}

void control_servo(unsigned int iPos0, unsigned int iPos1, unsigned int iPos2, SendIJogPacket *stSendIJOGPacket)
{
  unsigned char ucDOF = 3;
  printf("iPos0 = %d\n", iPos0);

  //	printf("LSB = %d\n", LSB(iPos));
  //	printf("MSB = %d\n", MSB(iPos));

  stSendIJOGPacket->ucHeader[0] = HEADER;
  stSendIJOGPacket->ucHeader[1] = HEADER;

  stSendIJOGPacket->ucPacketSize = MIN_PACKET_SIZE + CMD_I_JOG_STRUCT_SIZE * ucDOF;
  stSendIJOGPacket->ucChipID = BROADCAST_ID;
  stSendIJOGPacket->ucCmd = CMD_I_JOG;

  // set position (servo0)
  stSendIJOGPacket->ucData[0] = LSB(iPos0);
  stSendIJOGPacket->ucData[1] = MSB(iPos0);
  stSendIJOGPacket->ucData[2] = 0;
  stSendIJOGPacket->ucData[3] = 0;
  stSendIJOGPacket->ucData[4] = 50;

  // set position (servo1)
  stSendIJOGPacket->ucData[5] = LSB(iPos1);
  stSendIJOGPacket->ucData[6] = MSB(iPos1);
  stSendIJOGPacket->ucData[7] = 0;
  stSendIJOGPacket->ucData[8] = 1;
  stSendIJOGPacket->ucData[9] = 50;

  // set position (servo2)
  stSendIJOGPacket->ucData[10] = LSB(iPos2);
  stSendIJOGPacket->ucData[11] = MSB(iPos2);
  stSendIJOGPacket->ucData[12] = 0;
  stSendIJOGPacket->ucData[13] = 2;
  stSendIJOGPacket->ucData[14] = 50;

  // CheckSum
  stSendIJOGPacket->ucCheckSum1 = stSendIJOGPacket->ucPacketSize ^ stSendIJOGPacket->ucChipID ^ stSendIJOGPacket->ucCmd;
  for (unsigned char i = 0; i < CMD_I_JOG_STRUCT_SIZE * ucDOF; i++)
    stSendIJOGPacket->ucCheckSum1 ^= stSendIJOGPacket->ucData[i];

  stSendIJOGPacket->ucCheckSum2 = ~(stSendIJOGPacket->ucCheckSum1);
  stSendIJOGPacket->ucCheckSum1 &= CHKSUM_MASK;
  stSendIJOGPacket->ucCheckSum2 &= CHKSUM_MASK;

  printf("Succeed to set msg!\n");
}

int main(void)
{
  //--------------------------------------------------------------------------
  int fd;

  struct termios newtio;

  // open device
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);  // 192.168.4.168->ttyUSB0

  memset(&newtio, 0, sizeof(newtio));

  // control terminal
  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;  // parity error is ignored
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  //--------------------------------------------------------------------------
  SendRWPacket stSendRWPacket;
  SendIJogPacket stSendIJOGPacket;

  int iPos[3] = {0};  // array for servo position

  //--------------------------------------------------------------------------
  // make and send RAM_WRITE packet for servo0(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 0;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);

  //--------------------------------------------------------------------------
  // make and send RAM_WRITE packet for servo1(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 1;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);

  //--------------------------------------------------------------------------
  // make and send RAM_WRITE packet for servo2(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 2;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);

  //--------------------------------------------------------------------------
  // make and send RAM_WRITE packet for servo3(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 3;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);

  // initialize poosition
  control_servo(INITIAL_POS, INITIAL_POS, INITIAL_POS, &stSendIJOGPacket);
  write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);
  sleep(1);

  //--------------------------------------------------------------------------
  while (1)
  {
    printf("Press Enter key to continue!(press ESC and Enter to quit)\n");
    if (getchar() == 0x1b)
      break;

    for (int i = 0; i < 3; i++)
    {
      printf("input joint angle(degree)!ID%d:", i);
      scanf("%u", &iPos[i]);
      iPos[i] = degToPos(iPos[i]);
    }

    control_servo(iPos[0], iPos[1], iPos[2], &stSendIJOGPacket);
    write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);
  }

  printf("Press Enter key to terminate...\n");
  getchar();

  close(fd);

  return 0;
}
