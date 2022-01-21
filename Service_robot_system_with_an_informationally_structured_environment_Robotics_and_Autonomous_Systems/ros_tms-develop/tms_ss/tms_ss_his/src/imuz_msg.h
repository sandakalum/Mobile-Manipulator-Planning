#ifndef __IMUZ_MSG_H__
#define __IMUZ_MSG_H__

//#include <windows.h>
#include "typedef.h"

typedef WORD BTMSG_ID;

//
#define BTMSG_ID_EC ('E' << 8 | 'C')
#define BTMSG_ID_MT ('M' << 8 | 'T')
#define BTMSG_ID_ST ('S' << 8 | 'T')
#define BTMSG_ID_DP ('D' << 8 | 'P')

// CANメッセージのID
#define MSGID_ECHO 0x01
#define MSGID_ACC 0x02
#define MSGID_GYRO 0x03
#define MSGID_COMPASS 0x04
#define MSGID_STATUS 0x05
#define MSGID_SET_ROLE 0x06
#define MSGID_SET_PERIOD 0x07
#define MSGID_SET_RANGE 0x08
#define MSGID_SET_NODE_NO 0x09
#define MSGID_SAVE 0x0a
#define MSGID_RESET_TIMESTAMP 0x0b
#define MSGID_SET_MEASUREMENT_STATE 0x0c
#define MSGID_DEVICE_PROFILE 0x0d
#define MSGID_FACTORY_RESET 0x0e
#define MSGID_SET_BINARY 0x0f

#endif
