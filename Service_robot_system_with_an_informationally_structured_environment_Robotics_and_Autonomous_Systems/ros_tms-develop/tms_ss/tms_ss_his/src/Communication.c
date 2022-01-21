/*
 * @file
 * @brief	IMU−Z 通信
 * @author	瀬川正樹
 * @date	2010-07-06
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */

#include "Communication.h"
#include <unistd.h>

static HANDLE s_hCom = NULL;
static WORD s_cnt = 0;
static BOOL s_bRecover = FALSE;

/*
 * 1パケット(18byte)のデータを受信します
 */
#define PACKET_SIZE 18
BOOL com_ReadPacket(int fd, PBYTE lpBuff, DWORD readMax)
{
  DWORD dw;
  BYTE c;
  PBYTE p = lpBuff + s_cnt;

  if (readMax < PACKET_SIZE)
  {
    return FALSE;
  }

  while (read(fd, &c, 1))
  {
    if (s_bRecover)
    {
      if (c == 'B' || c == 'E')
      {
        s_bRecover = FALSE;
        p = lpBuff;
        *p++ = c;
        s_cnt = 1;
      }
      else
      {
      }
    }
    else
    {
      *p++ = c;
      s_cnt++;
      if (s_cnt >= PACKET_SIZE)
      {
        int i;
        BYTE sum = 0;
        for (i = 0; i < PACKET_SIZE - 1; i++)
        {
          sum += lpBuff[i];
        }
        if (sum == lpBuff[PACKET_SIZE - 1])
        {
          s_cnt = 0;
          return TRUE;
        }
        else
        {
          s_bRecover = TRUE;
          s_cnt = 0;
          return FALSE;
        }
      }
    }
  }
  return FALSE;
}
