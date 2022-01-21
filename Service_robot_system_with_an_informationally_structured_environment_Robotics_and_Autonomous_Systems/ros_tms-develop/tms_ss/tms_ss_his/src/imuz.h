/*
 * @file
 * @brief	IMU−Z データ構造体と定数
 * @author	瀬川正樹
 * @date	2010-07-06
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */
#ifndef __IMUZ_H__
#define __IMUZ_H__

//#include <windows.h>
#include "typedef.h"

/// <summary>
/// ロール。通信ノードとしての役割。
/// </summary>
/// <remarks>
/// IMU-Zは、複数台数で使用する場合に、様々な接続形態をとることができる。
/// それぞれのIMU-Zは、
/// Bluetoothを使用するか、CAN通信をまとめてBluetoothに転送するか、CANを使用するか、
/// の3種のいずれかとなる。
/// </remarks>
typedef enum tag_Role
{
  /// <summary>
  /// シングル(Bluetooth接続)
  /// </summary>
  SingleBT = 0,
  /// <summary>
  /// CANマスタ(Bluetooth接続あり)
  /// </summary>
  CanMasterBT,
  /// <summary>
  /// CANスレーブ
  /// </summary>
  CanSlave,
} Role;

typedef enum tag_AccelerometorRange
{
  /// <summary>
  /// レンジ±2[g]。感度1[mg/bit]
  /// </summary>
  Range2g_Sensitivity1mg = 0,
  /// <summary>
  /// レンジ±4[g]。感度2[mg/bit]
  /// </summary>
  Range4g_Sensitivity2mg,
  /// <summary>
  /// レンジ±8[g]。感度3.9[mg/bit]
  /// </summary>
  Range8g_Sensitivity3_9mg,
} AccelerometorRange;

/// <summary>
/// ジャイロセンサのレンジ設定
/// </summary>
/// <remarks>
/// IMU-Zに搭載のジャイロセンサは測定のレンジと感度を2段階に切り替えられる。
/// </remarks>
typedef enum tag_GyroscopeRange
{
  /// <summary>
  /// レンジ±500[deg/sec]。感度0.3[deg/bit]。
  /// </summary>
  Range500dps_Sensitivity0_3dps = 0,
  /// <summary>
  /// レンジ±2000[deg/sec]。感度1.2[deg/bit]。
  /// </summary>
  Range2000dps_Sensitivity1_2dps,

} GyroscopeRange;

/// <summary>
/// 地磁気センサのレンジ設定
/// </summary>
/// <remarks>
/// IMU-Zに搭載の地磁気センサは測定のレンジと感度を7段階に切り替えられる。
/// </remarks>
typedef enum tag_CompassRange
{
  /// <summary>
  /// レンジ±0.5[gauss]
  /// </summary>
  Range0_7Ga = 0,
  /// <summary>
  /// レンジ±1.0[gauss]
  /// </summary>
  Range1_0Ga,
  /// <summary>
  /// レンジ±1.5[gauss]
  /// </summary>
  Range1_5Ga,
  /// <summary>
  /// レンジ±2.0[gauss]
  /// </summary>
  Range2_0Ga,
  /// <summary>
  /// レンジ±3.2[gauss]
  /// </summary>
  Range3_2Ga,
  /// <summary>
  /// レンジ±3.8[gauss]
  /// </summary>
  Range3_8Ga,
  /// <summary>
  /// レンジ±4.5[gauss]
  /// </summary>
  Range4_5Ga,

} CompassRange;

typedef struct tag_MeasurementData
{
  /// <summary>
  /// ノード番号
  /// </summary>
  UINT node_no;
  /// <summary>
  /// タイムスタンプ。単位は[msec]。IMU-Zの起動時からカウントアップする、
  /// または、ResetTimestampメッセージによってリセットされカウントアップする値。
  /// 最大値は32bitの最大値(約50日)。
  /// </summary>
  UINT time;
  /// <summary>
  /// 加速度センサの値3軸分。加速度、単位は[g]。
  /// 配列は順に、x,y,z軸の値を示す。
  /// </summary>
  double acc[3];
  /// <summary>
  /// ジャイロセンサの値3軸分。角速度、単位は[deg/sec]。
  /// 配列は順に、x,y,z軸の値を示す。
  /// </summary>
  double gyro[3];
  /// <summary>
  /// 地磁気センサの値3軸分。磁束密度、単位は[gauss]。
  /// 配列は順に、x,y,z軸の値を示す。
  /// </summary>
  double comp[3];

} MeasurementData;

/// <summary>
/// IMU-Zステータスデータ
/// </summary>
/// <remarks>
/// IMU-Zの現在の状態と、内部で保存している設定値を通知するためのデータ
/// </remarks>
typedef struct tag_StatusData
{
  /// <summary>
  /// ノード番号
  /// </summary>
  UINT node_no;
  /// <summary>
  /// ロール。ノードの役割をしめす。
  /// ロールによって通信機能が選択される。
  /// </summary>
  Role role;
  /// <summary>
  /// 計測更新間隔。
  /// IMU-Zはこれで指定された時間間隔で、すべてのセンサをスキャンして値を取得して値を送信する。
  /// </summary>
  UINT period;
  /// <summary>
  /// 加速度センサのレンジの設定。3段階。
  /// </summary>
  AccelerometorRange range_acc;
  /// <summary>
  /// ジャイロセンサのレンジの設定。2段階。
  /// </summary>
  GyroscopeRange range_gyro;
  /// <summary>
  /// 地磁気センサのレンジの設定。7段階。
  /// </summary>
  CompassRange range_comp;
  /// <summary>
  /// 現在のバッテリーレベル。
  /// </summary>
  double batt;
  /// <summary>
  /// 現在のメッセージフォーマット。
  /// </summary>
  BOOL binary;

} StatusData;

/// <summary>
/// デバイスプロフィールデータ
/// </summary>
/// <remarks>
/// IMU-Z固有の情報を通知するためのデータ。
/// </remarks>
typedef struct tag_ProfileData
{
  /// <summary>
  /// ノード番号
  /// </summary>
  UINT node_no;
  /// <summary>
  /// ハードウェアリビジョン
  /// </summary>
  UINT hardware;
  /// <summary>
  /// ファームウェアバージョン
  /// </summary>
  UINT firmware;
  // public byte[] bd_addr = new byte[6];
  /// <summary>
  /// BDアドレス。Bluetoothで使用する固有のアドレス値。
  /// </summary>
  TCHAR bd_addr[7];

} ProfileData;

#endif
