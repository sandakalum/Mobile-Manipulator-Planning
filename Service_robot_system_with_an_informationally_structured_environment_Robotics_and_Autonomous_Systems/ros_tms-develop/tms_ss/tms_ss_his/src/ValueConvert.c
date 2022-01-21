/*
 * @file
 * @brief	IMU−Z Rawデータ⇒実データ変換
 * @author	瀬川正樹
 * @date	2010-07-06
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */

#include "ValueConvert.h"

/*
 * レンジによって係数が違います
 * 全センサ分の係数を保持
 */
static float s_accRatio[29];
static float s_gyroRatio[29];
static float s_compRatio[29];

/*
 * 変換の初期値
 * 全センサ分の係数を保持
 */
VOID conv_Init()
{
  int i;
  for (i = 0; i < 29; i++)
  {
    // s_accRatio[i] = 1.0f;
    s_accRatio[i] = 3.9f;
    s_gyroRatio[i] = 1.0f;
    s_compRatio[i] = 1.0f / 1300;
  }
}

/*
 * レンジをセットします
 * これによってレンジが違っても正しい値に変換されます
 */
BOOL conv_SetSensorRange(UINT node_no, AccelerometorRange range_acc, GyroscopeRange range_gyro, CompassRange range_comp)
{
  int beg, end;
  int i;

  if (node_no == 31)
  {
    beg = 1;
    end = 28;
  }
  else
  {
    beg = (int)node_no;
    end = (int)node_no;
  }

  for (i = beg; i <= end; i++)
  {
    switch (range_acc)
    {
      case Range2g_Sensitivity1mg:
        s_accRatio[i] = 1.0f;
        break;
      case Range4g_Sensitivity2mg:
        s_accRatio[i] = 2.0f;
        break;
      case Range8g_Sensitivity3_9mg:
        s_accRatio[i] = 3.9f;
        break;
    }
    switch (range_gyro)
    {
      case Range500dps_Sensitivity0_3dps:
        s_gyroRatio[i] = 1.0f;
        break;
      case Range2000dps_Sensitivity1_2dps:
        s_gyroRatio[i] = 4.0f;
        break;
    }
    switch (range_comp)
    {
      case Range0_7Ga:
        s_compRatio[i] = 1.0f / 1620;
        break;
      case Range1_0Ga:
        s_compRatio[i] = 1.0f / 1300;
        break;
      case Range1_5Ga:
        s_compRatio[i] = 1.0f / 970;
        break;
      case Range2_0Ga:
        s_compRatio[i] = 1.0f / 780;
        break;
      case Range3_2Ga:
        s_compRatio[i] = 1.0f / 530;
        break;
      case Range3_8Ga:
        s_compRatio[i] = 1.0f / 460;
        break;
      case Range4_5Ga:
        s_compRatio[i] = 1.0f / 390;
        break;
    }
  }
  return TRUE;
}

/*
 * 計測データの変換
 */
MeasurementData conv_Measurement(int node_no, int time_stamp, int acc[3], int gyro[3], int comp[3])
{
  int i;
  MeasurementData mess;

  mess.node_no = node_no;
  mess.time = time_stamp * 10;
  for (i = 0; i < 3; i++)
  {
    mess.acc[i] = conv_acc(node_no, acc[i]);
  }
  for (i = 0; i < 3; i++)
  {
    mess.gyro[i] = conv_gyro(node_no, gyro[i]);
  }
  for (i = 0; i < 3; i++)
  {
    mess.comp[i] = conv_comp(node_no, comp[i]);
  }
  return mess;
}

/*
 * 加速度の変換
 */
float conv_acc(int node_no, int raw_acc)
{
  return raw_acc / (1.0f * 1000.0f) * s_accRatio[node_no];
}
/*
 * ジャイロの変換
 */
float conv_gyro(int node_no, int raw_gyro)
{
  return (raw_gyro * 2.5f / 4096) / (2.0f / 1000.0f) * s_gyroRatio[node_no];
}
/*
 * 地磁気の変換
 */
float conv_comp(int node_no, int raw_comp)
{
  return raw_comp * s_compRatio[node_no];
}
/*
 * バッテリーレベルの変換
 */
float conv_batt(int node_no, int raw_batt)
{
  return ((raw_batt * 16.0) * 2.5 / 4096) / (3.3 / (3.3 + 10));
}

/*
 * ステータス情報の変換
 */
StatusData conv_Status(int node_no, int role, int period, int range_acc, int range_gyro, int range_comp, int raw_batt,
                       int binary)
{
  StatusData stat;
  stat.node_no = node_no;
  stat.role = (Role)role;
  stat.period = period * 10;
  stat.range_acc = (AccelerometorRange)range_acc;
  stat.range_gyro = (GyroscopeRange)range_gyro;
  stat.range_comp = (CompassRange)range_comp;
  stat.batt = conv_batt(node_no, raw_batt);
  stat.binary = binary;
  return stat;
}
