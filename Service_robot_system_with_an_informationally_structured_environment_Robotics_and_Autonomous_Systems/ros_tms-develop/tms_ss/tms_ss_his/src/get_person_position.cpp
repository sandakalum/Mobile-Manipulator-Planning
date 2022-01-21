//------------------------------------------------------------------------------
// @file   : fss_person_trakcing.cpp
// @brief  : find footprint cluster by classify info and personTrajectory
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.11.01)
// @date   : 2012.11.01
//------------------------------------------------------------------------------

#include <stdio.h>
#include <pthread.h>

#include <time.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <list>
#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_ss/fss_tf_data.h>
#include <tms_msg_ss/fss_tf_datas.h>
#include <tms_msg_ss/fss_person_trajectory_data.h>
#include <tms_msg_ss/fss_detected_cluster_data.h>
#include <tms_msg_ss/fss_observed_datas.h>
#include "../../tms_ss_fss/tms_ss_fss_exe/include/tms_ss_fss_exe/Cluster.cpp"
#include "../../tms_ss_fss/tms_ss_fss_exe/include/tms_ss_fss_exe/common.h"
#include "../../tms_ss_fss/tms_ss_fss_exe/include/tms_ss_fss_exe/Cluster.h"
#include <fstream>
#include <sstream>

//--
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
//--

//------------------------------------------------------------------------------
using namespace std;

#define SCORE_METHOD 2          // 1:distance of cluster's center, 2:overlap of clusters' fields
#define SAMEPOINT 50            // [mm]
#define SAMECLUSTER 150         // [mm]
#define DRAGCLUSTER 500         // [mm]
#define HUMAN_STEP_WIDTH 800    // [mm]
#define FOOTPRINTSIZE_MIN 150   // [mm]
#define FOOTPRINTSIZE_MAX 1000  // [mm]
#define OVERLAPTIME_ROOM 0.2    // [sec]
#define SENSINGTIME_MIN 0.2     // [sec]
#define SENSINGTIME_MAX 5.0     // [sec]
#define INTENSITY_THRESH 3000   //
#define isSplit 1               // split drug cluster
#define isMerge 1               // execute merge cluster
#define isCheckTMS 0            // check TMS info about other object pos
#define isOutput 0
#define isPrint 0

//--
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
#define MAX_SENSOR_NO 4   // 28					// ９軸センサ最大個数
#define BEFORE 0          // fssPositionData の前回時刻情報
#define NOW 1             // fssPositionData の今回時刻情報

#define _POSIX_SOURCE 1  // POSIX準拠のソース
//--

#define MAX_SENSOR_DATA_NUM 400  // 一時保存データの最大数

#define THRESHOLD_STOP 0.0200    // 停止とみなす加速度差分の閾値
#define THRESHOLD_STOP_LENGTH 4  // 停止とみなす加速度の計測時間（データ数）

//------------------------------------------------------------------------------
// declaration of functions

void callback(const tms_msg_ss::fss_class_data::ConstPtr &msg);
float getLength(float x1, float y1, float x2, float y2);
float getScore(std::list< Cluster >::iterator it, const tms_msg_ss::fss_class_data::ConstPtr &msg, int gt_index);
double getTime(uint64_t time);

//--
int rcv9axis();
// VOID on_BinaryMessage(PBYTE data, char fileName[]);
VOID on_BinaryMessage(PBYTE data, char fileName[], int sensorNo);
//--

void storeData(int sensorNo, unsigned long long rtime, float data);
int maxEvalValueNo();

//--
/* ９軸センサ情報 */
struct nineAxisData
{
  double acc[3];
  double gyro[3];
  double comp[3];
} sensor[MAX_SENSOR_NO];
//--

struct tmpAccData
{
  unsigned long long rtime[MAX_SENSOR_DATA_NUM];
  double data[MAX_SENSOR_DATA_NUM];
} accZ[MAX_SENSOR_NO], diffAccZ[MAX_SENSOR_NO];

// struct tmpDiffAccData{
//	unsigned long long	rostime[MAX_SENSOR_DATA_NUM];
//	double				data[MAX_SENSOR_DATA_NUM];
//} diffAccZ[MAX_SENSOR_NO];

//------------------------------------------------------------------------------
// declaration of variables

int detectedClusterIDMax = -1;
int trajectoryTreeClusterIDMax = -1;
bool isSetGlobalStartTime = false;
ros::Time GLOBAL_START_TIME;
ros::Time GLOBAL_LAST_TIME;

ros::Subscriber rosSub;
ros::Publisher rosPersonTrajectory;
ros::Publisher rosDetectedCluster;

std::list< Cluster > sensingClusterList;
std::list< Cluster > preDetectedClusterList;
std::list< Cluster > detectedClusterList;

std::list< Node > trajectoryTree;
std::vector< vector< Cluster > > humanTrajectory;

FILE *fpresultPosition;     // 計測結果出力用ファイルポインタ
char fileNamePosition[19];  // 保存ファイル名（現在時刻を格納する yyyymmddhhmmss）
FILE *fpresult9axis;        // 計測結果出力用ファイルポインタ
char fileName9axis[19];     // 保存ファイル名（現在時刻を格納する yyyymmddhhmmss）

//--
static bool first = true;  // プログラム開始フラグ（９軸センサのtime_stampを揃えるために使用）
//--

int selectedSensorNo = 0;             // 評価値を計算した結果，足と対応付けられたセンサの番号
float evalValue[MAX_SENSOR_NO];       // 評価値
int diffAccFssLost[MAX_SENSOR_NO];    // 足が離れた時刻と加速度変化がゼロになった時刻の差
int diffAccFssDetect[MAX_SENSOR_NO];  // 足が着いた時刻と加速度変化がゼロになった時刻の差

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  /* スレッド用パラメータ */
  int handle;
  pthread_t id;

  /* スレッドの生成 */
  handle = pthread_create(&id, NULL, (void *(*)(void *))rcv9axis, NULL);

  /* 現在時刻取得用 */
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);

  //	/* 配列の初期化 */
  memset(fileNamePosition, 0x00, sizeof(fileNamePosition));
  //	memset( fileName9axis, 0x00, sizeof(fileName9axis) );
  //	memset( fileName2, 0x00, sizeof(fileName2) );
  memset(evalValue, 0x00, sizeof(evalValue));
  memset(diffAccFssLost, 0x00, sizeof(diffAccFssLost));
  memset(diffAccFssDetect, 0x00, sizeof(diffAccFssDetect));

  /* 計測結果出力用ファイル名の設定 */
  sprintf(fileNamePosition, "%4d%02d%02d%02d%02d%02dp.csv", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
          pnow->tm_hour, pnow->tm_min,
          pnow->tm_sec);  // 現在時刻をファイル名にする（ファイル名は yyyymmddhhmmss.csv になる）
  if ((fpresultPosition = fopen(fileNamePosition, "w")) == NULL)
  {
    printf("FILE OPEN ERROR 1\n");
    exit(-1);
  }
  fprintf(fpresultPosition, "lostTimeS,lostTimeE,x,y,predictedSensorNo,"
                            "diffAccFssDetect_0,diffAccFssDetect_1,diffAccFssDetect_2,diffAccFssDetect_3,"
                            "diffAccFssLost_0,diffAccFssLost_1,diffAccFssLost_2,diffAccFssLost_3\n");
  fclose(fpresultPosition);

  ros::init(argc, argv, "get_person_position");
  ros::NodeHandle nh;

  // rosSub = nh.subscribe("fss_class_data", 10, callback);
  rosSub = nh.subscribe("fss_unknown_class_data", 10, callback);

  ros::spin();

  return (0);
}

//------------------------------------------------------------------------------
// Definition of Functions
void callback(const tms_msg_ss::fss_class_data::ConstPtr &msg)
{
  if (!isSetGlobalStartTime)
  {
    GLOBAL_START_TIME = msg->tMeasuredTime;
    isSetGlobalStartTime = true;
  }
  GLOBAL_LAST_TIME = msg->tMeasuredTime;

  printf("----------------------\n");
  printf("time: %lf\n", getTime(GLOBAL_LAST_TIME.toNSec()));

  //--------------------------------------------------------------------------
  // create List of clusters if no detected clusters
  if (sensingClusterList.size() == 0)
  {
    // TODO:併合処理
    for (unsigned int i = 0; i < msg->iID.size(); i++)
    {
      Cluster tempCluster;
      tempCluster.initialize(msg, i);
      sensingClusterList.push_back(tempCluster);
    }
  }
  else
  {
    std::list< Matching > MatchingLanking;
    std::list< Cluster >::iterator it;

    unsigned int mt_index, gt_index;
    mt_index = gt_index = 0;

    //--------------------------------------------------------------------------
    // create MatchingLanking
    for (it = sensingClusterList.begin(); it != sensingClusterList.end(); it++)
    {
      for (gt_index = 0; gt_index < msg->iID.size(); gt_index++)
      {
        float score;
        Matching tempMatching;

        score = getScore(it, msg, gt_index);
        if (score > 0)
        {
          tempMatching.mt_index = mt_index;
          tempMatching.gt_index = gt_index;
          tempMatching.score = score;

          if (MatchingLanking.size() == 0)
          {
            MatchingLanking.push_back(tempMatching);
          }
          else
          {
            bool isInsert = false;
            std::list< Matching >::iterator m_it;
            for (m_it = MatchingLanking.begin(); m_it != MatchingLanking.end(); m_it++)
            {
              if (tempMatching.score > m_it->score)
              {
                isInsert = true;
                m_it = MatchingLanking.insert(m_it, tempMatching);
                break;
              }
            }
            if (m_it == MatchingLanking.end() && !isInsert)
            {
              MatchingLanking.push_back(tempMatching);
            }
          }
        }
      }
      mt_index++;
    }

    //--------------------------------------------------------------------------
    // Update clusters' data order by score of MatchingLanking
    std::list< Matching >::iterator m_it;

    // isMatch
    std::vector< bool > isMatch;
    for (unsigned int i = 0; i < msg->iID.size(); i++)
    {
      isMatch.push_back(false);
    }
    // isUpdate
    for (it = sensingClusterList.begin(); it != sensingClusterList.end(); it++)
    {
      it->isUpdate = false;
    }
    // update
    for (m_it = MatchingLanking.begin(); m_it != MatchingLanking.end(); m_it++)
    {
      int count;
      float mt_bool;
      std::list< Cluster >::iterator mt_it;

      count = 0;
      std::list< Cluster >::iterator it;
      for (it = sensingClusterList.begin(); it != sensingClusterList.end(); it++)
      {
        if (count == m_it->mt_index)
        {
          mt_it = it;
          mt_bool = it->isUpdate;
        }
        count++;
      }
      if (!isMatch[m_it->gt_index] && !mt_bool)
      {
        // update
        mt_it->update(msg, m_it->gt_index);
        isMatch[m_it->gt_index] = true;
      }
    }

    //----------------------------------------------------------------------------
    // Push lost Cluster to preDetectedClusterList
    for (it = sensingClusterList.begin(); it != sensingClusterList.end();)
    {
      if (!it->isUpdate)
      {
        Cluster tempCluster;
        tempCluster.copy(it, 0, it->fCenterX.size(), true);

        // push preDetectedClusterList
        preDetectedClusterList.push_back(tempCluster);
        it = sensingClusterList.erase(it);
        continue;
      }
      it++;
    }

    //--------------------------------------------------------------------------
    // add new Cluster to sensingClusterList
    // TODO:併合処理
    for (unsigned int i = 0; i < msg->iID.size(); i++)
    {
      if (!isMatch[i])
      {
        Cluster tempCluster;
        tempCluster.initialize(msg, i);
        sensingClusterList.push_back(tempCluster);
      }
    }
  }

  //----------------------------------------------------------------------------
  // Analyze preDetectedClusterList
  std::list< Cluster >::iterator it;
  for (it = preDetectedClusterList.begin(); it != preDetectedClusterList.end();)
  {
    //----------------------------------------------------------------------------
    // split cluster if distance of start pos between end pos is large
    if (isSplit)
    {
      float len;
      float startfCenterX, startfCenterY;
      float endfCenterX, endfCenterY;

      startfCenterX = it->fCenterX.front();
      startfCenterY = it->fCenterY.front();
      endfCenterX = it->fCenterX.back();
      endfCenterY = it->fCenterY.back();

      len = getLength(startfCenterX, startfCenterY, endfCenterX, endfCenterY);

      if (len > DRAGCLUSTER)
      {
        // split drug cluster
        float middlefCenterX, middlefCenterY;
        middlefCenterX = (startfCenterX + endfCenterX) / 2.0;
        middlefCenterY = (startfCenterY + endfCenterY) / 2.0;

        float minLen = FLT_MAX;
        int middleIndex;
        for (unsigned int i = 0; i < it->fCenterX.size(); i++)
        {
          float _x = it->fCenterX[i];
          float _y = it->fCenterY[i];
          float _len = getLength(_x, _y, middlefCenterX, middlefCenterY);
          if (_len < minLen)
          {
            minLen = _len;
            middleIndex = i;
          }
        }

        // 前半、後半を別のクラスタとしてpushする
        Cluster frontCluster, backCluster;
        frontCluster.copy(it, 0, middleIndex);
        backCluster.copy(it, middleIndex, it->fCenterX.size());

        preDetectedClusterList.push_back(frontCluster);
        preDetectedClusterList.push_back(backCluster);
        it = preDetectedClusterList.erase(it);
        continue;
      }
    }

    //----------------------------------------------------------------------------
    // delete cluster if sensingTime is very short
    uint64_t s_time, e_time;
    double sensingTime;
    s_time = it->tMeasuredTime.front().toNSec();
    e_time = it->tMeasuredTime.back().toNSec();
    sensingTime = double(e_time - s_time) / 1.0e9;
    it->sensingTime = sensingTime;
    if (sensingTime < SENSINGTIME_MIN)
    {
      it = preDetectedClusterList.erase(it);
      continue;
    }
    it++;
  }

  //--
  unsigned long long lostTimeS = 0;
  unsigned long long lostTimeE = 0;
  static int lostPositionX = 0;
  static int lostPositionY = 0;
  static int lostClusterSize = 0;
  if ((preDetectedClusterList.size() > 0) && (lostPositionX != (int)preDetectedClusterList.back().fCenterX.back()))
  {
    //	if( preDetectedClusterList.size() > 0 ){

    lostTimeS = preDetectedClusterList.back().tMeasuredTime.front().toNSec();
    lostTimeE = preDetectedClusterList.back().tMeasuredTime.back().toNSec();
    lostPositionX = (int)preDetectedClusterList.back().fCenterX.back();
    lostPositionY = (int)preDetectedClusterList.back().fCenterY.back();
    lostClusterSize = (int)preDetectedClusterList.back().fSize.back();

    /* 評価値計算 */
    int sensorNo = 0;
    int dataNo = 0;
    int lostTimeNo = 0;
    int detectTimeNo = 0;
    float lostVal = 1.0;
    float detectVal = 1.0;

    for (sensorNo = 0; sensorNo < MAX_SENSOR_NO; sensorNo++)
    {
      for (dataNo = 0; dataNo < MAX_SENSOR_DATA_NUM; dataNo++)
      {
        printf("dataNo:%d\nlostTimeS:%llu\nlostTimeE:%llu\n difftime:%llu\n", dataNo, lostTimeS, lostTimeE,
               diffAccZ[sensorNo].rtime[dataNo]);
      }
    }

    for (sensorNo = 0; sensorNo < MAX_SENSOR_NO; sensorNo++)
    {
      // 足が離れる瞬間に加速度変化がなければ評価値を下げる
      for (dataNo = (MAX_SENSOR_DATA_NUM - 1); dataNo >= 0; dataNo--)
      {
        if ((lostTimeE > diffAccZ[sensorNo].rtime[dataNo]))
        {
          lostTimeNo = dataNo;
          printf("lostTimeE:%llu diffTime:%llu lostTimeNo:%d\n", lostTimeE, diffAccZ[sensorNo].rtime[dataNo],
                 lostTimeNo);
          //					if(		(diffAccZ[sensorNo].data[dataNo]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo-1]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo-2]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo-3]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo-4]>THRESHOLD_STOP) ){
          if ((diffAccZ[sensorNo].data[dataNo] < THRESHOLD_STOP) &&
              (diffAccZ[sensorNo].data[dataNo - 1] < THRESHOLD_STOP))
          {
            lostVal = 0.5;
          }
          else
          {
            lostVal = 1.0;
          }

          /* 足が離れた時刻と加速度変化がゼロになった時刻の差を計算 */
          int cntLength = 0;
          int cntStop = 0;
          for (cntLength = dataNo; cntLength >= 0; cntLength--)
          {
            diffAccFssLost[sensorNo]++;
            if (diffAccZ[sensorNo].data[dataNo] > THRESHOLD_STOP)
            {
              cntStop = 0;
            }
            else
            {
              cntStop++;
              if (cntStop > THRESHOLD_STOP_LENGTH)
              {
                break;
              }
            }
          }

          break;
        }
      }
      // 足が着く瞬間に加速度変化がなければ評価値を下げる
      for (dataNo = (MAX_SENSOR_DATA_NUM - 1); dataNo >= 0; dataNo--)
      {
        if (lostTimeS > diffAccZ[sensorNo].rtime[dataNo])
        {
          detectTimeNo = dataNo;
          printf("lostTimeS:%llu diffTime:%llu detectTimeNo:%d\n", lostTimeS, diffAccZ[sensorNo].rtime[dataNo],
                 detectTimeNo);
          //					if(		(diffAccZ[sensorNo].data[dataNo]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo+1]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo+2]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo+3]>THRESHOLD_STOP)
          //						 || (diffAccZ[sensorNo].data[dataNo+4]>THRESHOLD_STOP) ){
          if ((diffAccZ[sensorNo].data[dataNo] < THRESHOLD_STOP) &&
              (diffAccZ[sensorNo].data[dataNo + 1] < THRESHOLD_STOP))
          {
            detectVal = 0.5;
          }
          else
          {
            detectVal = 1.0;
          }

          /* 足が着いた時刻と加速度変化がゼロになった時刻の差を計算 */
          int cntLength = 0;
          int cntStop = 0;
          for (cntLength = dataNo; cntLength < MAX_SENSOR_DATA_NUM; cntLength++)
          {
            diffAccFssDetect[sensorNo]++;
            if (diffAccZ[sensorNo].data[dataNo] > THRESHOLD_STOP)
            {
              cntStop = 0;
            }
            else
            {
              cntStop++;
              if (cntStop > THRESHOLD_STOP_LENGTH)
              {
                break;
              }
            }
          }

          break;
        }
      }
      printf("sensorNo:%d dTime:%d lTime:%d\n", sensorNo, detectTimeNo, lostTimeNo);
      // 足が着いてから離れるまでの加速度変化がない回数を合計する
      for (dataNo = detectTimeNo; dataNo < lostTimeNo; dataNo++)
      {
        if (diffAccZ[sensorNo].data[dataNo] < THRESHOLD_STOP)
        {
          evalValue[sensorNo] = evalValue[sensorNo] + 1.0;
        }
      }
      // 評価値を再計算する
      evalValue[sensorNo] = lostVal * detectVal * evalValue[sensorNo];
    }

    printf("preDetectedClusterList_X : %f\n", preDetectedClusterList.back().fCenterX.back());
    printf("preDetectedClusterList_Y : %f\n", preDetectedClusterList.back().fCenterY.back());
    printf("maxEvalValue : %d\n", maxEvalValueNo());

    unsigned long long time_now = ros::Time::now().toNSec();
    if ((fpresultPosition = fopen(fileNamePosition, "a+")) == NULL)
    {
      printf("FILE OPEN ERROR 2\n");
      exit(-1);
    }
    fprintf(fpresultPosition, "%llu,%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", lostTimeS, lostTimeE, lostPositionX,
            lostPositionY, maxEvalValueNo(), diffAccFssDetect[0], diffAccFssDetect[1], diffAccFssDetect[2],
            diffAccFssDetect[3], diffAccFssLost[0], diffAccFssLost[1], diffAccFssLost[2], diffAccFssLost[3]);
    fclose(fpresultPosition);
  }

  //	unsigned long long newTimeS = 0;
  //	unsigned long long newTimeE = 0;
  //	static int	newPositionX	= 0;
  //	static int	newPositionY	= 0;
  //	static int	newClusterSize		= 0;
  //	if(    ( sensingClusterList.size() > 0 )
  //		&& ( newPositionX != (int)sensingClusterList.back().fCenterX.back() ) ){
  ////	if( preDetectedClusterList.size() > 0 ){
  //
  //		newTimeS = sensingClusterList.back().tMeasuredTime.front().toNSec();
  //		newTimeE = sensingClusterList.back().tMeasuredTime.back().toNSec();
  //		newPositionX	= (int)sensingClusterList.back().fCenterX.back();
  //		newPositionY	= (int)sensingClusterList.back().fCenterY.back();
  //		newClusterSize		= (int)sensingClusterList.back().fSize.back();
  //
  //		printf( "sensingClusterList_X : %f\n", sensingClusterList.back().fCenterX.back() );
  //		printf( "sensingClusterList_Y : %f\n", sensingClusterList.back().fCenterY.back() );
  //
  //		//time_now = ros::Time::now().toNSec();
  //		if( (fpresult=fopen(fileNamePosition,"a+")) == NULL ){
  //			printf( "FILE OPEN ERROR\n" );
  //			exit( -1 );
  //		}
  //		fprintf( fpresult, "%llu,%llu,%d,%d,%d,new\n", newTimeS, newTimeE, newPositionX, newPositionY, newClusterSize );
  //		fclose( fpresult );
  //	}
  //--

  //
  //	static unsigned long long time_now = 0;
  //	static int	lastPositionX	= 0;
  //	static int	lastPositionY	= 0;
  //	static int	clusterSize		= 0;
  //	if(    ( preDetectedClusterList.size() > 0 )
  //		&& ( lastPositionX != (int)preDetectedClusterList.back().fCenterX.back() ) ){
  ////	if( preDetectedClusterList.size() > 0 ){
  //
  //		lastPositionX	= (int)preDetectedClusterList.back().fCenterX.back();
  //		lastPositionY	= (int)preDetectedClusterList.back().fCenterY.back();
  //		clusterSize		= (int)preDetectedClusterList.back().fSize.back();
  //
  ////        printf( "----\n" );
  //		printf( "preDetectedClusterList_X : %f\n", preDetectedClusterList.back().fCenterX.back() );
  //		printf( "preDetectedClusterList_Y : %f\n", preDetectedClusterList.back().fCenterY.back() );
  //
  //		time_now = ros::Time::now().toNSec();
  //    	if( (fpresult=fopen(fileNamePosition,"a+")) == NULL ){
  //    		printf( "FILE OPEN ERROR\n" );
  //    		exit( -1 );
  //    	}
  //		fprintf( fpresult, "%llu,%d,%d,%d\n", time_now, lastPositionX, lastPositionY, clusterSize );
  //		fclose( fpresult );
  //	}
}

float getLength(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float getScore(std::list< Cluster >::iterator it, const tms_msg_ss::fss_class_data::ConstPtr &msg, int gt_index)
{
  float score = 0;

  if (SCORE_METHOD == 1)
  {
    //------------------------------------------------------------------------------
    // distance of clusters' centers
    float distanceCenters =
        getLength(it->fCenterX.back(), it->fCenterY.back(), msg->fCenterX[gt_index], msg->fCenterY[gt_index]);
    if (distanceCenters > SAMECLUSTER)
      distanceCenters = 0;

    score = distanceCenters;
  }
  else if (SCORE_METHOD == 2)
  {
    //------------------------------------------------------------------------------
    // calculate overlap fields of clusters

    // get two vertex of Rectangle
    float RectangleMinX, RectangleMinY;
    float RectangleMaxX, RectangleMaxY;

    RectangleMinX = FLT_MAX;
    RectangleMinY = FLT_MAX;
    RectangleMaxX = FLT_MIN;
    RectangleMaxY = FLT_MIN;

    for (unsigned int i = 0; i < msg->LrfData[gt_index].fX2.size(); i++)
    {
      float x = msg->LrfData[gt_index].fX2[i];
      float y = msg->LrfData[gt_index].fY2[i];

      if (x < RectangleMinX)
        RectangleMinX = x;
      if (x > RectangleMaxX)
        RectangleMaxX = x;
      if (y < RectangleMinY)
        RectangleMinY = y;
      if (y > RectangleMaxY)
        RectangleMaxY = y;
    }

    // check overlap cluster fields
    float overlap_width, overlap_height;

    if (RectangleMinX < it->fRectangleMaxX.back() && it->fRectangleMinX.back() < RectangleMaxX)
    {
      if (RectangleMinY < it->fRectangleMaxY.back() && it->fRectangleMinY.back() < RectangleMaxY)
      {
        // calculate width, height of field
        overlap_width = overlap_height = FLT_MAX;
        float width[4], height[4];
        width[0] = it->fRectangleMaxX.back() - RectangleMinX;
        width[1] = RectangleMaxX - it->fRectangleMinX.back();
        width[2] = it->fRectangleMaxX.back() - it->fRectangleMinX.back();
        width[3] = RectangleMaxX - RectangleMinX;
        for (unsigned int i = 0; i < 4; i++)
        {
          if (width[i] < overlap_width)
            overlap_width = width[i];
        }

        height[0] = it->fRectangleMaxY.back() - RectangleMinY;
        height[1] = RectangleMaxY - it->fRectangleMinY.back();
        height[2] = it->fRectangleMaxY.back() - it->fRectangleMinY.back();
        height[3] = RectangleMaxY - RectangleMinY;
        for (unsigned int i = 0; i < 4; i++)
        {
          if (height[i] < overlap_height)
            overlap_height = height[i];
        }

        score = overlap_width * overlap_height;
        // printf("score: %f, [%f, %f][%f, %f] - [%f, %f][%f, %f]\n", score, it->fRectangleMinX.back(),
        // it->fRectangleMinY.back(), it->fRectangleMaxX.back(), it->fRectangleMaxY.back(), RectangleMinX,
        // RectangleMinY, RectangleMaxX, RectangleMaxY);
      }
    }
  }
  return score;
}

double getTime(uint64_t time)
{
  double _time;
  _time = (double)(time - GLOBAL_START_TIME.toNSec()) / 1.0e9;

  return _time;
}

//--
/******* main function *******/
int rcv9axis()
{
  struct termios oldtio, newtio;  // 仮想ポート制御用
  BYTE buff[BUFF_SIZE];           // ９軸センサから取得したパケットの一時格納先
  int fdSensor0 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor1 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor2 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdSensor3 = 0;              // ９軸センサ接続先仮想ポートのファイルディスクリプタ
  int fdMax = 0;                  // ファイルディスクリプタの最大値
  fd_set readfs;                  // ファイルディスクリプタの集合
  struct timeval Timeout;
  int res = 0;

  /* 現在時刻取得用 */
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);

  /* 配列の初期化 */
  memset(buff, 0x00, sizeof(buff));

  /* 計測結果出力用ファイル名の設定 */
  sprintf(fileName9axis, "%4d%02d%02d%02d%02d%02dn.csv", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
          pnow->tm_hour, pnow->tm_min,
          pnow->tm_sec);  // 現在時刻をファイル名にする（ファイル名は yyyymmddhhmmss.csv になる）
  if ((fpresult9axis = fopen(fileName9axis, "w")) == NULL)
  {
    printf("FILE OPEN ERROR 3\n");
    exit(-1);
  }
  fprintf(fpresult9axis, "rostime,time_stamp,node_no,"
                         "sensor.acc[0],sensor.acc[1],sensor.acc[2],"
                         "sensor.gyro[0],sensor.gyro[1],sensor.gyro[2],"
                         "sensor.comp[0],sensor.comp[1],sensor.comp[2],\n");
  fclose(fpresult9axis);

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

  /* time_stampを揃えるために一度実行 */
  /* sensor0 */
  while (true)
  {
    if (com_ReadPacket(fdSensor0, buff, BUFF_SIZE))
    {                                            // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName9axis, 0);  // パケット受信処理
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
    {                                            // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName9axis, 1);  // パケット受信処理
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
    {                                            // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName9axis, 2);  // パケット受信処理
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
    {                                            // パケットを１つ読み込む
      on_BinaryMessage(buff, fileName9axis, 3);  // パケット受信処理
      break;
    }
    else
    {
      usleep(10);  // パケット到着待ち
    }
  }
  first = false;

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
      {                                            // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName9axis, 0);  // パケット受信処理
      }
    }
    /* sensor1 */
    if (FD_ISSET(fdSensor1, &readfs))
    {
      if (com_ReadPacket(fdSensor1, buff, BUFF_SIZE))
      {                                            // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName9axis, 1);  // パケット受信処理
      }
    }
    /* sensor2 */
    if (FD_ISSET(fdSensor2, &readfs))
    {
      if (com_ReadPacket(fdSensor2, buff, BUFF_SIZE))
      {                                            // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName9axis, 2);  // パケット受信処理
      }
    }
    /* sensor3 */
    if (FD_ISSET(fdSensor3, &readfs))
    {
      if (com_ReadPacket(fdSensor3, buff, BUFF_SIZE))
      {                                            // パケットを１つ読み込む
        on_BinaryMessage(buff, fileName9axis, 3);  // パケット受信処理
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

/*** Bluetoothのバイナリメッセージデコード処理 ***/
VOID on_BinaryMessage(PBYTE data, char fileName[], int sensorNo)
{
  int axisNo = 0;  // 軸番号（0:x 1:y 2:z）
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
    //		printf( "node_no : %d\n", node_no );
    //		printf( " acc : %11.6lf %11.6lf %11.6lf\n",   sensor[node_no-1].acc[0],  sensor[node_no-1].acc[1],
    // sensor[node_no-1].acc[2]  );
    //		printf( "gyro : %11.6lf %11.6lf %11.6lf\n",   sensor[node_no-1].gyro[0], sensor[node_no-1].gyro[1],
    // sensor[node_no-1].gyro[2] );
    //		printf( "comp : %11.6lf %11.6lf %11.6lf\n\n", sensor[node_no-1].comp[0], sensor[node_no-1].comp[1],
    // sensor[node_no-1].comp[2] );
    // dataNo++;

    if (first == false)
    {  // 初回は記録しない
      time_now = ros::Time::now().toNSec() + ros::Duration(9 * 60 * 60).toNSec();
      if ((fpresult9axis = fopen(fileName9axis, "a+")) == NULL)
      {
        printf("FILE OPEN ERROR 4\n");
        exit(-1);
      }
      fprintf(fpresult9axis, "%llu,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time_now, time_stamp, node_no,
              sensor[node_no - 1].acc[0], sensor[node_no - 1].acc[1], sensor[node_no - 1].acc[2],
              sensor[node_no - 1].gyro[0], sensor[node_no - 1].gyro[1], sensor[node_no - 1].gyro[2],
              sensor[node_no - 1].comp[0], sensor[node_no - 1].comp[1], sensor[node_no - 1].comp[2]);
      //			fprintf( fpresult, "%d,%lf,%lf,%lf,", time_stamp, sensor[node_no-1].acc[0], sensor[node_no-1].acc[1],
      // sensor[node_no-1].acc[2] );
      fclose(fpresult9axis);

      /* データを配列に格納 */
      // 配列の最後の要素にデータを格納する．
      // 要素番号が小さいほど古いデータになる．
      // 配列が埋まっていたら古いデータを消去し，最後の要素にデータを格納する．
      storeData(sensorNo, time_now, sensor[node_no - 1].acc[2]);
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
//--

void storeData(int sensorNo, unsigned long long rtime, float data)
{
  //	FILE	*fpresult2;			// 計測結果出力用ファイルポインタ
  //	char	fileName2[19];			// 保存ファイル名（現在時刻を格納する yyyymmddhhmmss）

  static int dataNum[4];        // accZ に格納したデータ数 dataNum[0] がセンサ0に対応
  static bool firstRun = true;  // この関数の初回実行フラグ
  int dataNo = 0;               // データ番号

  //	/* 現在時刻取得用 */
  //	time_t now = time(NULL);
  //	struct tm *pnow = localtime(&now);

  /* 配列初期化 */
  if (firstRun == true)
  {
    firstRun = false;
    memset(dataNum, 0x00, sizeof(dataNum));

    //		/* 計測結果出力用ファイル名の設定 */
    //		memset( fileName2, 0x00, sizeof(fileName2) );
    //		sprintf(fileName2, "%4d%02d%02d%02d%02d%02dt.csv",
    //			pnow->tm_year+1900,
    //			pnow->tm_mon+1,
    //			pnow->tm_mday,
    //			pnow->tm_hour,
    //			pnow->tm_min,
    //			pnow->tm_sec
    //		);		// 現在時刻をファイル名にする（ファイル名は yyyymmddhhmmss.csv になる）
    //		if( (fpresult2=fopen(fileName2,"w")) == NULL ){
    //			printf( "FILE OPEN ERROR 5\n" );
    //			exit( -1 );
    //		}
    //		fprintf( fpresult2, "rostime_accZ,data_accZ,rostime_diff,data_diff\n" );
    //		fclose( fpresult2 );
  }

  /* データを配列に格納 */
  // 配列の最後の要素にデータを格納する．
  // 要素番号が小さいほど古いデータになる．
  // 配列が埋まっていたら古いデータを消去し，最後の要素にデータを格納する．
  if (dataNum[sensorNo] > MAX_SENSOR_DATA_NUM)
  {
    for (dataNo = 0; dataNo < (MAX_SENSOR_DATA_NUM - 1); dataNo++)
    {
      accZ[sensorNo].rtime[dataNo] = accZ[sensorNo].rtime[dataNo + 1];
      accZ[sensorNo].data[dataNo] = accZ[sensorNo].data[dataNo + 1];
      diffAccZ[sensorNo].rtime[dataNo] = diffAccZ[sensorNo].rtime[dataNo + 1];
      diffAccZ[sensorNo].data[dataNo] = diffAccZ[sensorNo].data[dataNo + 1];
    }
    accZ[sensorNo].rtime[MAX_SENSOR_DATA_NUM - 1] = rtime;
    accZ[sensorNo].data[MAX_SENSOR_DATA_NUM - 1] = data;
    diffAccZ[sensorNo].rtime[MAX_SENSOR_DATA_NUM - 1] = rtime;
    diffAccZ[sensorNo].data[MAX_SENSOR_DATA_NUM - 1] =
        fabs(accZ[sensorNo].data[MAX_SENSOR_DATA_NUM - 1] - accZ[sensorNo].data[MAX_SENSOR_DATA_NUM - 2]);
  }
  else
  {
    dataNum[sensorNo] = dataNum[sensorNo] + 1;
    accZ[sensorNo].rtime[dataNum[sensorNo] - 1] = rtime;
    accZ[sensorNo].data[dataNum[sensorNo] - 1] = data;
    if (dataNum[sensorNo] >= 2)
    {  // ２個以上データが格納されていたら前回との差分をとる
      diffAccZ[sensorNo].rtime[dataNum[sensorNo] - 1] = rtime;
      diffAccZ[sensorNo].data[dataNum[sensorNo] - 1] =
          fabs(accZ[sensorNo].data[dataNum[sensorNo] - 1] - accZ[sensorNo].data[dataNum[sensorNo] - 2]);
    }
  }
  //	for( dataNo = 0; dataNo < dataNum[sensorNo]; dataNo++ ){
  //		printf( "%d:%llu,%f,%llu,%f\n", sensorNo, accZ[sensorNo].rtime[dataNo], accZ[sensorNo].data[dataNo],
  // diffAccZ[sensorNo].rtime[dataNo], diffAccZ[sensorNo].data[dataNo] );
  //	}

  //	/* 評価値更新 */
  //	evalValue[sensorNo] = 0;
  //	for( dataNo = 0; dataNo < dataNum[sensorNo]; dataNo++ ){
  //		if( diffAccZ[sensorNo].data[dataNo] < THRESHOLD_STOP ){
  //			evalValue[sensorNo] = evalValue[sensorNo] + 1;
  //		}
  //	}

  //	if( (fpresult2=fopen(fileName2,"a+")) == NULL ){
  //		printf( "FILE OPEN ERROR 6\n" );
  //		exit( -1 );
  //	}
  //	fprintf( fpresult2, "--\n" );
  //	for( dataNo = 0; dataNo < dataNum[sensorNo]; dataNo++ ){
  //		fprintf( fpresult2, "%llu,%f,%llu,%f\n",
  //			accZ[sensorNo].time[dataNo], accZ[sensorNo].data[dataNo], diffAccZ[sensorNo].time[dataNo],
  // diffAccZ[sensorNo].data[dataNo]
  //		);
  //	}
  //	fclose( fpresult2 );
}

int maxEvalValueNo(void)
{
  int sensorNo = 0;
  float maxValue = 0.0;
  int maxNo = 0;
  //	bool	sameValue	= false;

  for (sensorNo = 0; sensorNo < MAX_SENSOR_NO; sensorNo++)
  {
    printf("%f ", evalValue[sensorNo]);
    //		if( evalValue[sensorNo] == maxValue ){
    //			sameValue = true;
    //			break;
    //		}
    if (evalValue[sensorNo] > maxValue)
    {
      maxNo = sensorNo;
      maxValue = evalValue[sensorNo];
    }
  }
  //	printf( "\n" );
  //	if( sameValue == true ){
  //		return( 99 );
  //	}else{
  return (maxNo);
  //	}
}
