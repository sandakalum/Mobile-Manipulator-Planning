#ifndef CONTROL_ARM_HPP_
#define CONTROL_ARM_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <math.h>
#include <cmath>
#include <tms_msg_rc/kobuki_control_1.h>
#include <geometry_msgs/Twist.h>

// SERVO control specific includes
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>

int fd;
int p;

// Index
#define PROTOCOL_SIZE_IDX 2
#define PROTOCOL_ID_IDX 3
#define PROTOCOL_CMD_IDX 4
#define PROTOCOL_CS1_IDX 5
#define PROTOCOL_CS2_IDX 6
#define PROTOCOL_DATA_IDX 7

// HEADER
#define HEADER 0xFF

// SIZE
#define MIN_PACKET_SIZE 7
#define MIN_ACK_PACKET_SIZE 9
#define MAX_PACKET_SIZE 223
#define MAX_DATA_SIZE (MAX_PACKET_SIZE - MIN_PACKET_SIZE)

// ID
#define MAX_ID 0xFD
#define BROADCAST_ID 0xFE

// CMD - Request Packet
#define CMD_EEP_WRITE 0x01
#define CMD_EEP_READ 0x02
#define CMD_RAM_WRITE 0x03
#define CMD_RAM_READ 0x04
#define CMD_RW_DATA_ADDR_IDX 7
#define CMD_RW_DATA_LEN_IDX 8
#define CMD_I_JOG 0x05
#define CMD_I_JOG_STRUCT_SIZE 5
#define CMD_I_JOG_MAX_DRS (MAX_DATA_SIZE / CMD_I_JOG_STRUCT_SIZE)
#define CMD_S_JOG 0x06
#define CMD_S_JOG_STRUCT_SIZE 4
#define CMD_S_JOG_MAX_DRS (MAX_DATA_SIZE / CMD_S_JOG_STRUCT_SIZE)
#define CMD_STAT 0x07
#define CMD_ROLLBACK 0x08
#define CMD_REBOOT 0x09

#define CMD_MIN (CMD_EEP_WRITE)
#define CMD_MAX (CMD_REBOOT)

// CMD - ACK Packet
#define CMD_ACK_MASK 0x40

#define CMD_EEP_WRITE_ACK (CMD_EEP_WRITE | CMD_ACK_MASK)
#define CMD_EEP_READ_ACK (CMD_EEP_READ | CMD_ACK_MASK)
#define CMD_RAM_WRITE_ACK (CMD_RAM_WRITE | CMD_ACK_MASK)
#define CMD_RAM_READ_ACK (CMD_RAM_READ | CMD_ACK_MASK)
#define CMD_I_JOG_ACK (CMD_I_JOG | CMD_ACK_MASK)
#define CMD_S_JOG_ACK (CMD_S_JOG | CMD_ACK_MASK)
#define CMD_STAT_ACK (CMD_STAT | CMD_ACK_MASK)
#define CMD_ROLLBACK_ACK (CMD_ROLLBACK | CMD_ACK_MASK)
#define CMD_REBOOT_ACK (CMD_REBOOT | CMD_ACK_MASK)

#define CMD_ACK_MIN (CMD_EEP_WRITE_ACK)
#define CMD_ACK_MAX (CMD_REBOOT_ACK)

#define CHKSUM_MASK 0xFE

// Position Range
#define INITIAL_POS 512

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

SendRWPacket stSendRWPacket;
SendIJogPacket stSendIJOGPacket;

typedef struct DrsJog
{
  unsigned int uiValue : 15;
  unsigned int reserved : 1;
} DrsJog;

typedef struct DrsSet
{
  unsigned char ucStopFlag : 1;
  unsigned char ucMode : 1;
  unsigned char ucLedGreen : 1;
  unsigned char ucLedBlue : 1;
  unsigned char ucLedRed : 1;
  unsigned char ucJogInvalid : 1;
  unsigned char reserved : 2;
} DrsSet;

typedef struct DrsIJog
{
  DrsJog stJog;
  DrsSet stSet;
  unsigned char ucId;
  unsigned char ucPlayTime;
} DrsIJog;

typedef struct DrsSJog
{
  DrsJog stJog;
  DrsSet stSet;
  unsigned char ucId;
} DrsSJog;

typedef struct DrsIJogData
{
  DrsIJog stIJog[CMD_I_JOG_MAX_DRS];
} DrsIJogData;

typedef struct DrsSJogData
{
  unsigned char ucPlayTime;
  DrsSJog stSJog[CMD_S_JOG_MAX_DRS];
} DrsSJogData;

typedef struct DrsRWData
{
  unsigned char ucAddress;
  unsigned char ucLen;
  unsigned char ucData[MAX_DATA_SIZE - 2];
} DrsRWData;

typedef union DrsData
{
  unsigned char ucData[MAX_PACKET_SIZE - MIN_PACKET_SIZE];

  DrsRWData stRWData;
  DrsIJogData stIJogData;
  DrsSJogData stSJogData;
} DrsData;

typedef struct
{
  unsigned char ucHeader[2];
  unsigned char ucPacketSize;
  unsigned char ucChipID;
  unsigned char ucCmd;
  unsigned char ucCheckSum1;
  unsigned char ucCheckSum2;
  DrsData unData;
} DrsPacket;

enum
{
  DRS_RXWAITING,
  DRS_RXCOMPLETE,
  DRS_HEADERNOTFOUND,
  DRS_INVALIDSIZE,
  DRS_UNKNOWNCMD,
  DRS_INVALIDID,
  DRS_CHKSUMERROR,
  DRS_RXTIMEOUT
} DrsRxStatus;

void control_manipulator(unsigned int th0, unsigned int th1, unsigned int th2, unsigned int th3,
                         SendIJogPacket *stSendIJOGPacket);

#ifdef GRASP_C
#define GRASP_EXT
#else
#define GRASP_EXT extern
#endif

#endif /* CONTROL_ARM_HPP_ */
