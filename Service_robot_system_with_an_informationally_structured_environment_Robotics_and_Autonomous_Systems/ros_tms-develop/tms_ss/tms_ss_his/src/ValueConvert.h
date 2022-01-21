/*
 * @file
 * @brief	IMU−Z Rawデータ⇒実データ変換
 * @author	瀬川正樹
 * @date	2010-07-06
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */
#ifndef __VALUE_CONVERT_H__
#define __VALUE_CONVERT_H__

#include "imuz.h"
//#include <windows.h>
#include "typedef.h"

VOID conv_Init();
BOOL conv_SetSensorRange(UINT node_no, AccelerometorRange range_acc, GyroscopeRange range_gyro,
                         CompassRange range_comp);
MeasurementData conv_Measurement(int node_no, int time_stamp, int acc[3], int gyro[3], int comp[3]);
StatusData conv_Status(int node_no, int role, int period, int range_acc, int range_gyro, int range_comp, int raw_batt,
                       int binary);

float conv_acc(int node_no, int raw_acc);
float conv_gyro(int node_no, int raw_gyro);
float conv_comp(int node_no, int raw_comp);
float conv_batt(int node_no, int raw_batt);

#endif
