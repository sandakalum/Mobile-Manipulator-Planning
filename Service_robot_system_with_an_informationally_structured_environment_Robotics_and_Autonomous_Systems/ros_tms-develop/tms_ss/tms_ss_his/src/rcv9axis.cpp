#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include "kbhit.h"
#include "imuz.h"
#include "imuz_msg.h"
#include "Communication.h"
#include "ValueConvert.h"
#include "Communication.c"
#include "ValueConvert.c"

//#include <boost/date_time/local_time/local_time.hpp>
//#include <ros/ros.h>
//#include "tms_db_msgs/tmsdb_data.h"
//#include "tms_db_msgs/tmsdb_get_person_info.h"

/*** 通信ポート設定 ********************************************
  このプログラムを実行するPCに接続したBluetoothアダプタが
  どの仮想ポートに割り当てられているかを設定する．
***************************************************************/
//#define COM_PORT_NAME	"COM4"				// COM10未満の場合
#define COM_PORT_NAME "¥¥¥¥.¥¥COM15"  // COM10以上の場合（例えばCOM13なら "¥¥¥¥.¥¥COM13" とする）

#define BUFF_SIZE 200                 // ９軸センサからのパケットを格納するバッファのサイズ
#define BAUDRATE B9600                // ９軸センサとの通信速度
#define SENSOR_PORT_0 "/dev/rfcomm0"  // Bluetoothアダプタの仮想ポートをbindしたポート名
#define SENSOR_PORT_1 "/dev/rfcomm1"  // Bluetoothアダプタの仮想ポートをbindしたポート名
#define SENSOR_PORT_2 "/dev/rfcomm2"  // Bluetoothアダプタの仮想ポートをbindしたポート名
#define SENSOR_PORT_3 "/dev/rfcomm3"  // Bluetoothアダプタの仮想ポートをbindしたポート名

#define THSTOP 0.050      // 躍度（加加速度）がこの値以下なら停止と判断する
#define MAX_PERSON_NO 10  // 追跡可能人数
//#define MAX_POSITION_NO		100000000				// 人位置最大保存個数
#define MAX_SENSOR_NO 28  // ９軸センサ最大個数
//#define MAX_SENSOR_DATA_NO	100000000				// ９軸センサデータ最大個数
#define BEFORE 0  // fssPositionData の前回時刻情報
#define NOW 1     // fssPositionData の今回時刻情報

#define _POSIX_SOURCE 1  // POSIX準拠のソース

/* FSSから得られる人位置情報 */
struct fssPositionData
{
  unsigned long rostime;
  unsigned int id;
  float x;
  float y;
  float z;
  float theta;
  unsigned int state;
  unsigned int place;
} position, tmpPosition[2];
//} position[MAX_POSITION_NO], tmpPosition[2];

/* ９軸センサ情報 */
struct nineAxisData
{
  double acc[3];
  double gyro[3];
  double comp[3];
} sensor[MAX_SENSOR_NO];
//} sensor[MAX_SENSOR_NO][MAX_SENSOR_DATA_NO];

static int dataNo = 0;     // センサデータ番号（プログラム開始時から常に増加）
static bool first = true;  // プログラム開始フラグ（９軸センサのtime_stampを揃えるために使用）

/*** Bluetoothのバイナリメッセージデコード処理 ***/
VOID on_BinaryMessage(PBYTE data, char fileName[])
{
  int axisNo = 0;  // 軸番号（0:x 1:y 2:z）
  FILE *fpresult;  // 計測結果出力用ファイルポインタ
  static int cntStop[3] = {
      0, 0, 0};  // 歩行状態がどのくらい続いているかを計数（実際にはセンサが固定された方の足が動いている時間を計数）
  static int cntWalk[3] = {
      0, 0, 0};                                // 停止状態がどのくらい続いているかを計数（実際にはセンサが固定された方の足が止まっている時間を計数）
  static int isWalk[3] = {0, 0, 0};            // 歩行状態フラグ（実際にはセンサが固定された方の足が動いているのを確認するフラグ）
  static double areaAcc[3] = {0.0, 0.0, 0.0};  // 躍度の積分値（加速度）
  static int time_stamp_first[MAX_SENSOR_NO];  // 計測開始時刻

  /* 計測データ */
  int node_no = 0;                               // ９軸センサのID
  int time_stamp = 0;                            // 計測時刻
  int raw_acc[3];                                // 加速度
  int raw_gyro[3];                               // 角速度
  int raw_comp[3];                               // 地磁気
  static double accBefore[3] = {0.0, 0.0, 0.0};  // 前回時刻の加速度

  /* ステータス情報 */
  int role = 0;
  int period = 0;
  int range_acc = 0;
  int range_gyro = 0;
  int raw_batt = 0;
  int range_comp = 0;
  int binary = 0;

  static bool firstRun = true;  // この関数の初回実行フラグ
  unsigned long long time_now;

  ProfileData prof;      // デバイスプロファイル情報
  MeasurementData mess;  // 生データから物理量へ変換した後の計測データ
  StatusData stat;       // 生データから変換後のステータス情報

  /* 配列初期化 */
  memset(raw_acc, 0x00, sizeof(raw_acc));
  memset(raw_gyro, 0x00, sizeof(raw_gyro));
  memset(raw_comp, 0x00, sizeof(raw_comp));
  if (firstRun == true)
  {
    memset(time_stamp_first, 0x00, sizeof(time_stamp_first));
    firstRun = false;
  }

  if (data[0] == 'B')
  {
    node_no = (UINT)data[1] >> 3;

    if (first == true)
    {  // 計測開始時のtime_stampを保持
      time_stamp_first[node_no] =
          ((UINT)data[1] << 12) & 0x7000 | ((UINT)data[2] << 4) & 0x0ff0 | ((UINT)data[3] >> 4) & 0x000f;
    }
    else
    {  // ２回目以降は計測開始時のtime_stampを引く（各９軸センサのスイッチを入れるタイミングがずれてもtime_stampは揃う）
      time_stamp = ((UINT)data[1] << 12) & 0x7000 | ((UINT)data[2] << 4) & 0x0ff0 | ((UINT)data[3] >> 4) & 0x000f;
      time_stamp = time_stamp - time_stamp_first[node_no];
    }

    raw_acc[0] = ((int)data[3] << 8) & 0x0f00 | data[4];
    raw_acc[1] = ((int)data[5] << 4) | (data[6] >> 4) & 0x0f;
    raw_acc[2] = ((int)data[6] << 8) & 0x0f00 | data[7];
    raw_gyro[0] = ((int)data[8] << 4) | (data[9] >> 4) & 0x0f;
    raw_gyro[1] = ((int)data[9] << 8) & 0x0f00 | data[10];
    raw_gyro[2] = ((int)data[11] << 4) | (data[12] >> 4) & 0x0f;
    raw_comp[0] = ((int)data[12] << 8) & 0x0f00 | data[13];
    raw_comp[1] = ((int)data[14] << 4) | (data[15] >> 4) & 0x0f;
    raw_comp[2] = ((int)data[15] << 8) & 0x0f00 | data[16];
    for (axisNo = 0; axisNo < 3; axisNo++)
    {
      if (raw_acc[axisNo] > 2047)
      {
        raw_acc[axisNo] -= 4096;
      }
      if (raw_gyro[axisNo] > 2047)
      {
        raw_gyro[axisNo] -= 4096;
      }
      if (raw_comp[axisNo] > 2047)
      {
        raw_comp[axisNo] -= 4096;
      }
    }

    mess = conv_Measurement(node_no, time_stamp, raw_acc, raw_gyro, raw_comp);

    for (axisNo = 0; axisNo < 3; axisNo++)
    {
      sensor[node_no - 1].acc[axisNo] = mess.acc[axisNo];    // node_noは１から始まる
      sensor[node_no - 1].gyro[axisNo] = mess.gyro[axisNo];  // node_noは１から始まる
      sensor[node_no - 1].comp[axisNo] = mess.comp[axisNo];  // node_noは１から始まる
    }
    printf("node_no : %d\n", node_no);
    printf(" acc : %11.6lf %11.6lf %11.6lf\n", sensor[node_no - 1].acc[0], sensor[node_no - 1].acc[1],
           sensor[node_no - 1].acc[2]);
    printf("gyro : %11.6lf %11.6lf %11.6lf\n", sensor[node_no - 1].gyro[0], sensor[node_no - 1].gyro[1],
           sensor[node_no - 1].gyro[2]);
    printf("comp : %11.6lf %11.6lf %11.6lf\n\n", sensor[node_no - 1].comp[0], sensor[node_no - 1].comp[1],
           sensor[node_no - 1].comp[2]);
    // dataNo++;

    if (first == false)
    {  // 初回は記録しない
      if ((fpresult = fopen(fileName, "a+")) == NULL)
      {
        printf("FILE OPEN ERROR\n");
        exit(-1);
      }
      fprintf(fpresult, "%llu,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time_now, time_stamp, node_no,
              sensor[node_no - 1].acc[0], sensor[node_no - 1].acc[1], sensor[node_no - 1].acc[2],
              sensor[node_no - 1].gyro[0], sensor[node_no - 1].gyro[1], sensor[node_no - 1].gyro[2],
              sensor[node_no - 1].comp[0], sensor[node_no - 1].comp[1], sensor[node_no - 1].comp[2]);
      //			fprintf( fpresult, "%d,%lf,%lf,%lf,", time_stamp, sensor[node_no-1].acc[0], sensor[node_no-1].acc[1],
      // sensor[node_no-1].acc[2] );
      fclose(fpresult);
    }
  }
  else if (data[0] == 'E')
  {
    if ((data[1] == 'E') && (data[2] == 'C'))
    {
      /* エコーの応答 */
      node_no = data[3];
      // display_Echo(node_no);
    }
    else if ((data[1] == 'S') && (data[2] == 'T'))
    {
      /* ステータス情報 */
      node_no = data[3];
      role = data[4];
      period = (int)data[5] << 8 | data[6];
      range_acc = data[7];
      range_gyro = data[8];
      raw_batt = data[9];
      range_comp = data[10];
      binary = data[11];

      stat = conv_Status(node_no, role, period, range_acc, range_gyro, range_comp, raw_batt, binary);
      // display_Status(stat);
    }
    else if ((data[1] == 'D') && (data[2] == 'P'))
    {
      /* デバイスプロファイル情報 */
      prof.node_no = data[3];
      prof.hardware = data[4];
      prof.firmware = data[5];
      sprintf(prof.bd_addr, "%02x%02x%02x%02x%02x%02x", data[6], data[7], data[8], data[9], data[10], data[11]);
      // display_ProfileData(prof);
    }
  }
}

/******* main function *******/
int main(int argc, char **argv)
{
  struct termios oldtio, newtio;  // 仮想ポート制御用
  BYTE buff[BUFF_SIZE];           // ９軸センサから取得したパケットの一時格納先
  char command[255];              // ９軸センサへ送信するコマンド
  char fileName[14];              // 保存ファイル名（現在時刻を格納する yyyymmddhhmmss）
  int fdSensor0 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor1 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor2 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor3 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  FILE *fpSensor0;                // ９軸センサへのファイルポインタ
  int tmpPersonNo = 0;            // DBから人位置を取得する際のとりあえずの番号
  FILE *fpresult;                 // 計測結果出力用ファイルポインタ
  int positionNo = 0;             // 人位置番号
  int fdMax = 0;                  // ファイルディスクリプタの最大値
  fd_set readfs;                  // ファイルディスクリプタの集合
  struct timeval Timeout;
  int res = 0;
  struct sigaction saio;
  char measuredTime[14];  // 計測時刻 yyyymmddhhmmss

  /* 現在時刻取得用 */
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);

  /* 配列の初期化 */
  memset(buff, 0x00, sizeof(buff));
  memset(command, 0x00, sizeof(command));
  memset(fileName, 0x00, sizeof(fileName));
  memset(measuredTime, 0x00, sizeof(measuredTime));

  //	ros::init(argc, argv, "rcv9axis");
  //	ros::NodeHandle n;
  //
  //	ros::ServiceClient client;
  //	tms_db_msgs::tmsdb_get_person_info srv;
  //
  //	client = n.serviceClient<tms_db_msgs::tmsdb_get_person_info>("tmsdb_get_current_person_info");

  /* 計測結果出力用ファイル名の設定 */
  sprintf(fileName, "%4d%02d%02d%02d%02d%02d.csv", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour,
          pnow->tm_min,
          pnow->tm_sec);  // 現在時刻をファイル名にする（ファイル名は yyyymmddhhmmss.csv になる）
  if ((fpresult = fopen(fileName, "w")) == NULL)
  {
    printf("FILE OPEN ERROR\n");
    exit(-1);
  }
  //	fprintf( fpresult,	"rostime,x,y,time_stamp,node_no,"
  //						"sensor.acc[0],sensor.acc[1],sensor.acc[2],"
  //						"sensor.gyro[0],sensor.gyro[1],sensor.gyro[2],"
  //						"sensor.comp[0],sensor.comp[1],sensor.comp[2]\n");
  fprintf(fpresult, "rostime,time_stamp,node_no,"
                    "sensor.acc[0],sensor.acc[1],sensor.acc[2],"
                    "sensor.gyro[0],sensor.gyro[1],sensor.gyro[2],"
                    "sensor.comp[0],sensor.comp[1],sensor.comp[2],\n");
  //	fprintf( fpresult,	"time,"
  //						"time_stamp1,sensor1.acc[0],sensor1.acc[1],sensor1.acc[2],"
  //						"time_stamp2,sensor2.acc[0],sensor2.acc[1],sensor2.acc[2],"
  //						"time_stamp3,sensor3.acc[0],sensor3.acc[1],sensor3.acc[2],"
  //						"time_stamp4,sensor4.acc[0],sensor4.acc[1],sensor4.acc[2]\n"
  //	);
  fclose(fpresult);

  /*** メッセージ変換の初期化 ******************************************
    ValueConvert.c で定義している conv_Init() 関数内の以下の値を
    実際に使用するスケールに合わせて変更する
    ＊変更する値
      s_accRatio[i]		// 加速度センサのスケール
      s_gyroRatio[i]		// 角速度センサのスケール
      s_compRatio[i]		// 地磁気センサのスケール
  *********************************************************************/
  conv_Init();

  /* ポートのオープン */
  /* sensor0 */
  fdSensor0 = open(SENSOR_PORT_0, O_RDWR | O_NOCTTY);
  if (fdSensor0 < 0)
  {
    perror(SENSOR_PORT_0);
    exit(-1);
  }
  /* ポートの設定 */
  /* sensor0 */
  tcgetattr(fdSensor0, &oldtio);  // 現在のポート設定を待避
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  /* ポート設定の有効化 */
  /* sensor0 */
  tcflush(fdSensor0, TCIFLUSH);
  tcsetattr(fdSensor0, TCSANOW, &newtio);

  /* sensor1 */
  fdSensor1 = open(SENSOR_PORT_1, O_RDWR | O_NOCTTY);
  if (fdSensor1 < 0)
  {
    perror(SENSOR_PORT_1);
    exit(-1);
  }
  /* sensor1 */
  tcgetattr(fdSensor1, &oldtio);  // 現在のポート設定を待避
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;  // キャラクタ間タイマは未使用
  newtio.c_cc[VMIN] = 0;   // １文字受け取るまでブロックする
  /* sensor1 */
  tcflush(fdSensor1, TCIFLUSH);
  tcsetattr(fdSensor1, TCSANOW, &newtio);

  /* ポートのオープン */
  /* sensor2 */
  fdSensor2 = open(SENSOR_PORT_2, O_RDWR | O_NOCTTY);
  if (fdSensor2 < 0)
  {
    perror(SENSOR_PORT_2);
    exit(-1);
  }
  /* ポートの設定 */
  /* sensor2 */
  tcgetattr(fdSensor2, &oldtio);  // 現在のポート設定を待避
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  /* ポート設定の有効化 */
  /* sensor2 */
  tcflush(fdSensor2, TCIFLUSH);
  tcsetattr(fdSensor2, TCSANOW, &newtio);

  /* ポートのオープン */
  /* sensor3 */
  fdSensor3 = open(SENSOR_PORT_3, O_RDWR | O_NOCTTY);
  if (fdSensor3 < 0)
  {
    perror(SENSOR_PORT_3);
    exit(-1);
  }
  /* ポートの設定 */
  /* sensor3 */
  tcgetattr(fdSensor3, &oldtio);  // 現在のポート設定を待避
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  /* ポート設定の有効化 */
  /* sensor3 */
  tcflush(fdSensor3, TCIFLUSH);
  tcsetattr(fdSensor3, TCSANOW, &newtio);

  ////////
  if (fdSensor0 > fdMax)
  {
    fdMax = fdSensor0;
  }
  if (fdSensor1 > fdMax)
  {
    fdMax = fdSensor1;
  }
  if (fdSensor2 > fdMax)
  {
    fdMax = fdSensor2;
  }
  if (fdSensor3 > fdMax)
  {
    fdMax = fdSensor3;
  }
  fdMax = fdMax + 1;
  ////////

  /* time_stampを揃えるために一度実行 */
  /* sensor0 */
  while (true)
  {
    if (com_ReadPacket(fdSensor0, buff, BUFF_SIZE))
    {                                    // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName);  // パケット受信処理
      break;
    }
    else
    {
      usleep(10);  // パケット到着待ち
    }
  }
  /* sensor1 */
  while (true)
  {
    if (com_ReadPacket(fdSensor1, buff, BUFF_SIZE))
    {                                    // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName);  // パケット受信処理
      break;
    }
    else
    {
      usleep(10);  // パケット到着待ち
    }
  }
  /* sensor2 */
  while (true)
  {
    if (com_ReadPacket(fdSensor2, buff, BUFF_SIZE))
    {                                    // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName);  // パケット受信処理
      break;
    }
    else
    {
      usleep(10);  // パケット到着待ち
    }
  }
  /* sensor3 */
  while (true)
  {
    if (com_ReadPacket(fdSensor3, buff, BUFF_SIZE))
    {                                    // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName);  // パケット受信処理
      break;
    }
    else
    {
      usleep(10);  // パケット到着待ち
    }
  }
  first = false;

  //	unsigned long long time_now = ros::Time::now().toNSec();
  //	if( (fpresult=fopen(fileName,"a+")) == NULL ){
  //		printf( "FILE OPEN ERROR\n" );
  //		exit( -1 );
  //	}
  //	fprintf( fpresult, "%llu\n", time_now );
  //	fclose( fpresult );

  int i = 0;

  // while( true ){
  while (!kbhit())
  {
    ////////
    FD_SET(fdSensor0, &readfs);
    FD_SET(fdSensor1, &readfs);
    FD_SET(fdSensor2, &readfs);
    FD_SET(fdSensor3, &readfs);
    Timeout.tv_usec = 20000;
    Timeout.tv_sec = 0;
    res = select(fdMax, &readfs, NULL, NULL, &Timeout);
    if (res == 0)
    {
      i++;
      if ((i % 10000) == 0)
      {
        printf("timeout[%d]\n", (i / 10000));
      }
    }
    ////////

    /* sensor0 */
    if (FD_ISSET(fdSensor0, &readfs))
    {
      if (com_ReadPacket(fdSensor0, buff, BUFF_SIZE))
      {                                    // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName);  // パケット受信処理
      }
    }
    /* sensor1 */
    if (FD_ISSET(fdSensor1, &readfs))
    {
      if (com_ReadPacket(fdSensor1, buff, BUFF_SIZE))
      {                                    // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName);  // パケット受信処理
      }
    }
    /* sensor2 */
    if (FD_ISSET(fdSensor2, &readfs))
    {
      if (com_ReadPacket(fdSensor2, buff, BUFF_SIZE))
      {                                    // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName);  // パケット受信処理
      }
    }
    /* sensor3 */
    if (FD_ISSET(fdSensor3, &readfs))
    {
      if (com_ReadPacket(fdSensor3, buff, BUFF_SIZE))
      {                                    // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName);  // パケット受信処理
      }
    }
  }

  /* ポートのクローズ */
  close(fdSensor0);
  close(fdSensor1);
  close(fdSensor2);
  close(fdSensor3);

  return (0);
}
