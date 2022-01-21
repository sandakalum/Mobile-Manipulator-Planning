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

// turtlebot arm data(cm)
#define L1 9.75000    // length from servo0 to servo1
#define L2 10.512000  // length from servo1 to servo2
#define L3 9.51000    // length from servo2 to servo3

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

// convert radian to degree
double radToDeg(double radian)
{
  return (radian * 180 / M_PI);
}

// convert degree to position
int degToPos(double degree)
{
  return int(3.0722 * degree + 235.5);
}

// compute for inverse kinematics
// input: goal position coodinate (Px, Py, Pz)
// output:joint angles (Th0, Th1, Th2) radian→degree→position
void compute_ik(int Px, int Py, int Pz, int* Th0, int* Th1, int* Th2)
{
  double l = 0;    // length from O to goal position
  double phy = 0;  // angle betweeen X and goal position
  double tmp = 0;

  double th0, th1 = 0;
  l = sqrt((Px * Px) + (Py * Py));
  printf("l = %f", l);
  phy = atan2(Py, Px);
  printf("phy = %f", phy);

  tmp = (L1 * L1 + l * l - L2 * L2) / (2 * L1 * l);
  printf("x_temp = %f", tmp);
  // goal Th0, Th1, Th2
  th0 = phy - acos(tmp);
  printf("rad = %f", th0);
  th0 = radToDeg(th0);
  printf("deg = %f", th0);
  *Th0 = degToPos(th0);
  printf("pos = %d\n", *Th0);

  tmp = (L1 * L1 + L2 * L2 - l * l) / (2 * L1 * L2);
  printf("y_temp = %f", tmp);
  th1 = M_PI - acos(tmp);
  printf("rad = %f", th1);
  th1 = radToDeg(th1) + 90;
  printf("deg = %f", th1);
  *Th1 = degToPos(th1);
  printf("pos = %d\n", *Th1);

  /*グリッパは-90度で(上から)把持すると仮定*/
  //	tmp = Pz/L3;
  //	printf("Pz/L3=%f", tmp);
  //	th2 = asin(tmp);
  //	printf("rad=%f", th2);
  //	th2 = radToDeg(th2);
  //	printf("deg=%f", th2);
  *Th2 = 777;
  printf("pos=%d\n", *Th2);
}

void control_single_servo(unsigned int id, unsigned int iPos3, SendIJogPacket* stSendIJOGPacket)
{
  unsigned char ucDOF = 1;
  printf("iPos3 = %d\n", iPos3);

  stSendIJOGPacket->ucHeader[0] = HEADER;
  stSendIJOGPacket->ucHeader[1] = HEADER;

  stSendIJOGPacket->ucPacketSize = MIN_PACKET_SIZE + CMD_I_JOG_STRUCT_SIZE * ucDOF;
  stSendIJOGPacket->ucChipID = BROADCAST_ID;
  stSendIJOGPacket->ucCmd = CMD_I_JOG;

  // set position (servo3)
  stSendIJOGPacket->ucData[0] = LSB(iPos3);
  stSendIJOGPacket->ucData[1] = MSB(iPos3);
  stSendIJOGPacket->ucData[2] = 0;
  stSendIJOGPacket->ucData[3] = id;
  stSendIJOGPacket->ucData[4] = 200;

  // CheckSum
  stSendIJOGPacket->ucCheckSum1 = stSendIJOGPacket->ucPacketSize ^ stSendIJOGPacket->ucChipID ^ stSendIJOGPacket->ucCmd;
  for (unsigned char i = 0; i < CMD_I_JOG_STRUCT_SIZE * ucDOF; i++)
    stSendIJOGPacket->ucCheckSum1 ^= stSendIJOGPacket->ucData[i];

  stSendIJOGPacket->ucCheckSum2 = ~(stSendIJOGPacket->ucCheckSum1);
  stSendIJOGPacket->ucCheckSum1 &= CHKSUM_MASK;
  stSendIJOGPacket->ucCheckSum2 &= CHKSUM_MASK;

  printf("Succeed to set msg!\n");
}

void control_servo(unsigned int iPos0, unsigned int iPos1, unsigned int iPos2, SendIJogPacket* stSendIJOGPacket)
{
  unsigned char ucDOF = 4;
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
  stSendIJOGPacket->ucData[4] = 200;

  // set position (servo1)
  stSendIJOGPacket->ucData[5] = LSB(iPos1);
  stSendIJOGPacket->ucData[6] = MSB(iPos1);
  stSendIJOGPacket->ucData[7] = 0;
  stSendIJOGPacket->ucData[8] = 1;
  stSendIJOGPacket->ucData[9] = 200;

  // set position (servo2)
  stSendIJOGPacket->ucData[10] = LSB(iPos2);
  stSendIJOGPacket->ucData[11] = MSB(iPos2);
  stSendIJOGPacket->ucData[12] = 0;
  stSendIJOGPacket->ucData[13] = 2;
  stSendIJOGPacket->ucData[14] = 200;

  // set position (servo3)
  stSendIJOGPacket->ucData[15] = LSB(512);
  stSendIJOGPacket->ucData[16] = MSB(512);
  stSendIJOGPacket->ucData[17] = 0;
  stSendIJOGPacket->ucData[18] = 3;
  stSendIJOGPacket->ucData[19] = 200;

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

  double Px, Py = 0;      // for servo goal position
  int Th0, Th1, Th2 = 0;  // for joint angle(21~1002)

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
  control_servo(INITIAL_POS, INITIAL_POS, INITIAL_POS_Z, &stSendIJOGPacket);
  write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);
  sleep(1);

  //--------------------------------------------------------------------------
  while (1)
  {
    printf("Press Enter key to continue!(press ESC and Enter to quit)\n");
    if (getchar() == 0x1b)
      break;

    printf("input goal position x_coordinates:");
    scanf("%lf", &Px);  // expect undigned int
    printf("input goal position y_coordinates:");
    scanf("%lf", &Py);  // expect undigned int

    compute_ik(Px, Py, 0, &Th0, &Th1, &Th2);
    control_servo(Th0, Th1, Th2, &stSendIJOGPacket);
    write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);

    control_single_servo(2, 780, &stSendIJOGPacket);
    write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);

    sleep(5);
    control_single_servo(3, 760, &stSendIJOGPacket);
    write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);

    sleep(5);
    control_single_servo(2, INITIAL_POS_Z, &stSendIJOGPacket);
    write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);
  }

  printf("Press Enter key to terminate...\n");
  getchar();

  close(fd);

  return 0;
}
