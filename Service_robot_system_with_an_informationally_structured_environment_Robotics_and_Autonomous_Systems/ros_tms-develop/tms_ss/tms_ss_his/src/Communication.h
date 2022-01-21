/*
 * @file
 * @brief	IMU−Z 通信
 * @author	瀬川正樹
 * @date	2010-07-06
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */
#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "typedef.h"
#include <unistd.h>
#include <stdio.h>

// BOOL com_Open(LPCTSTR comName);
// VOID com_Close();
// BOOL com_ReadLine(PBYTE lpBuff, DWORD readMax);
BOOL com_ReadPacket(FILE *fprfcomm0, PBYTE lpBuff, DWORD readMax);
// BOOL com_Send(const PBYTE lpBuff, DWORD size);
// BOOL com_SendString(LPCTSTR line);

#endif
