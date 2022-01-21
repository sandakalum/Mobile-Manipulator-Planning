#pragma once

#define MAX_CONNECT 4
#define MAX_DATA_SIZE_LRF 2000
#define MAX_TRACKING_OBJECT 20

#define MIN_DIFF_DIST 0.02          // 0.02  // 前景とみなす背景からの距離（Gaussianでは使わない）
#define MIN_OBJ_PROB 0.00000000001  // 0.005  // 前景とみなす確率（Gaussianで使う）
//#define MIN_OBJ_PROB 0.005
#define N_RING 1000               // Gaussianの計算に使うデータ数

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define deg2rad(x) ((double)x * M_PI / 180.0)
#define rad2deg(x) ((double)x * 180.0 / M_PI)

typedef struct _LRFParam
{
  double tx;
  double ty;
  double tz;
  double rx;
  double ry;
  double rz;
} LRFParam;

#define ZeroMemory(arg1, arg2) memset(arg1, 0, arg2)
#define CopyMemory(arg1, arg2, arg3) memcpy(arg1, arg2, arg3)

#include "config.h"
