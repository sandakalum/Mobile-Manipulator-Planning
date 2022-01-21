//------------------------------------------------------------------------------
// @file   : bas_person.cpp
// @brief  : analyze human behavior by trajectory, position of furnitures
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.11.13)
// @date   : 2012.11.13
//------------------------------------------------------------------------------

#include <time.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <list>
#include <tms_msg_ss/fss_person_trajectory_data.h>
#include <tms_msg_ss/fss_detected_cluster_data.h>
#include <tms_msg_ss/fss_observed_datas.h>
#include <tms_msg_ss/bas_behavior_data.h>
#include <fstream>
#include <sstream>

#include <tms_sa_bas_person/common.h>
#include <tms_sa_bas_person/HumanModel.h>

//------------------------------------------------------------------------------
using namespace std;

#define WALKINGMODEL_METHOD 2  // 1: calculate ave, var ofgaitCycle	2: set popular ave, var
#define OUTLIER_N 2.0          // σ
#define POPULAR_GAITCYCLE 1.6  // [sec]
#define START_SLEEPING 5.0     // [sec]
#define AREA_CHAIR 1000        // [mm]
#define EntrancePointX 1000    // [mm]
#define EntrancePointY 0       // [mm]
#define isOutput 0

//------------------------------------------------------------------------------
// declaration of functions

int getEstimatedState(int t, int preState, int walkingBehavior, int sittingBehavior, int sleepingBehavior);
void callbackChairBuffer(const tms_msg_ss::fss_observed_datas::ConstPtr &msg);
void callbackTrajectory(const tms_msg_ss::fss_person_trajectory_data::ConstPtr &msg);
void callbackDetectedCluster(const tms_msg_ss::fss_detected_cluster_data::ConstPtr &msg);
void estimateWalkingBehavior();
void estimateSittingBehavior();
void estimateSleepingBehavior();
void makeBehaviorResult();
void mergeBehavior();
bool isExit(int t);
bool getBedPos(ros::Time time, float &bedX, float &bedY);
bool getChairPos(ros::Time time, float &chairX, float &chairY);
bool isNearBed(float bedX, float bedY, float centerX, float centerY);
bool isNearChair(float chairX, float chairY, float centerX, float centerY);
float getLength(float x1, float y1, float x2, float y2);
double getTime(uint64_t time);

void displayWalkingHistory();
void displaySleepingHistory();

void outputLogfile();
void outputNearChairData();
void outputNearBedData();
void outputGaitCycleData();
void outputStepCycleData();
void outputHumanBehaviorData();
void outputFootPrintListData();
void outputFootPrintAllData();
void outputDetectedClusterListData();

// massage, service
ros::Subscriber rosSubTrajectory;
ros::Subscriber rosSubDetectedCluster;
ros::Subscriber rosSubChairBuffer;
ros::Publisher rosPubHumanBehavior;

//------------------------------------------------------------------------------
// declaration of variables
ros::Time GLOBAL_START_TIME;
ros::Time GLOBAL_LAST_TIME;
bool isSetGlobalStartTime = false;

std::vector< SumSensingTime > nearBedList;
std::vector< SumSensingTime > nearChairList;
tms_msg_ss::fss_observed_datas m_msgChairBuffer;
tms_msg_ss::fss_person_trajectory_data m_msgPersonTrajectory;
tms_msg_ss::fss_detected_cluster_data m_msgDetectedClusterList;

// behavior
HumanModel human;
std::vector< HumanBehavior > walkingHistory;
std::vector< HumanBehavior > sittingHistory;
std::vector< HumanBehavior > sleepingHistory;
std::vector< vector< int > > behaviorResult;

//------------------------------------------------------------------------------
// output stream
time_t now;
std::ofstream ofs1;
char filename1[256];  // for NearChairData and chair position
std::ofstream ofs2;
char filename2[256];  // for NearBedData and bed position
std::ofstream ofs3;
char filename3[256];  // for GaitCycleData
std::ofstream ofs4;
char filename4[256];  // for HumanBehaviorData
std::ofstream ofs5;
char filename5[256];  // for FootPrintListData
std::ofstream ofs6;
char filename6[256];  // for FootPrintAllData
std::ofstream ofs7;
char filename7[256];  // for DetectedClusterListData

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  if (isOutput)
  {
    now = time(NULL);
    // output setting
    sprintf(filename1, "NearChairData_%ld.txt", now);
    sprintf(filename2, "NearBedData_%ld.txt", now);
    sprintf(filename3, "GaitCycleData_%ld.txt", now);
    sprintf(filename4, "HumanBehaviorData_%ld.txt", now);
    sprintf(filename5, "FootPrintListData_%ld.txt", now);
    sprintf(filename6, "FootPrintAllData_%ld.txt", now);
    sprintf(filename7, "DetectedClusterListData_%ld.txt", now);
    ofs1.open(filename1);
    ofs2.open(filename2);
    ofs3.open(filename3);
    ofs4.open(filename4);
    ofs5.open(filename5);
    ofs6.open(filename6);
    ofs7.open(filename7);
  }

  ros::init(argc, argv, "bas_person");
  ros::NodeHandle nh;

  rosSubTrajectory = nh.subscribe("fss_person_trajectory_data", 10, callbackTrajectory);
  rosSubDetectedCluster = nh.subscribe("fss_detected_cluster_data", 10, callbackDetectedCluster);
  rosSubChairBuffer = nh.subscribe("fss_chair_buffer_data", 10, callbackChairBuffer);
  rosPubHumanBehavior = nh.advertise< tms_msg_ss::bas_behavior_data >("bas_behavior_data", 10);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    // estimate
    estimateWalkingBehavior();
    estimateSittingBehavior();
    estimateSleepingBehavior();
    makeBehaviorResult();

    int behaviorWalking = 0;
    int behaviorSitting = 0;
    int behaviorSleeping = 0;
    int mergedBehavior = 0;

    if (walkingHistory.size() > 0)
      behaviorWalking = walkingHistory.back().behavior;
    if (sittingHistory.size() > 0)
      behaviorSitting = sittingHistory.back().behavior;
    if (sleepingHistory.size() > 0)
      behaviorSleeping = sleepingHistory.back().behavior;
    if (behaviorResult.size() > 0)
      mergedBehavior = behaviorResult.back()[3];

    printf("----------------------------\n");
    printf("time: %lf\n", getTime(GLOBAL_LAST_TIME.toNSec()));
    printf("personTrajectory: %ld\n", m_msgPersonTrajectory.trajectory.size());
    printf("detectedClusterList: %ld\n", m_msgDetectedClusterList.cluster.size());
    printf("------------------\n");
    printf("WalkingModel: %d\n", behaviorWalking);
    printf("SittingModel: %d\n", behaviorSitting);
    printf("SleepingModel: %d\n", behaviorSleeping);
    printf("mergedResult: %d\n", mergedBehavior);

    // displayWalkingHistory();
    // displaySleepingHistory();

    ros::spinOnce();
    loop_rate.sleep();
  }

  //------------------------------------------------------------------------------
  // for output
  if (isOutput)
  {
    printf("------\n");
    printf("Output files.\n");
    outputLogfile();

    ofs1.close();
    printf("-%s\n", filename1);
    ofs2.close();
    printf("-%s\n", filename2);
    ofs3.close();
    printf("-%s\n", filename3);
    ofs4.close();
    printf("-%s\n", filename4);
    ofs5.close();
    printf("-%s\n", filename5);
    ofs6.close();
    printf("-%s\n", filename6);
    ofs7.close();
    printf("-%s\n", filename7);
  }
  return (0);
}

//------------------------------------------------------------------------------
// Definition of Functions
void callbackTrajectory(const tms_msg_ss::fss_person_trajectory_data::ConstPtr &msg)
{
  GLOBAL_START_TIME = msg->tMeasuredStartTime;
  GLOBAL_LAST_TIME = msg->tMeasuredLastTime;

  m_msgPersonTrajectory = *msg;
}

void callbackDetectedCluster(const tms_msg_ss::fss_detected_cluster_data::ConstPtr &msg)
{
  m_msgDetectedClusterList = *msg;
}

void callbackChairBuffer(const tms_msg_ss::fss_observed_datas::ConstPtr &msg)
{
  m_msgChairBuffer = *msg;
}

float getLength(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double getTime(uint64_t time)
{
  double _time;
  _time = (double)(time - GLOBAL_START_TIME.toNSec()) / 1.0e9;

  return _time;
}

void estimateWalkingBehavior()
{
  // TODO:
  // 毎回全クリアせずに、差分の分析のみ行う処理に変更する必要あり
  human.clearAllParameter();
  walkingHistory.clear();

  //------------------------------------------------------------------------------
  // get GaitCycle
  human.behaviorWalking = BEHAVIOR_LOST;
  human.calcBaseStepCycle(m_msgPersonTrajectory);
  human.calcBaseGaitCycle(m_msgPersonTrajectory);
  human.calcBaseStepCycleByAll();
  human.calcBaseGaitCycleByAll();

  double threshold;
  if (WALKINGMODEL_METHOD == 1)
    threshold = human.baseGaitCycleByAll + OUTLIER_N * sqrt(human.baseGaitCycleVarByAll);
  else if (WALKINGMODEL_METHOD == 2)
    threshold = (double)POPULAR_GAITCYCLE;

  //------------------------------------------------------------------------------
  // make walkingHistory
  for (unsigned int i = 0; i < m_msgPersonTrajectory.trajectory.size(); i++)
  {
    for (unsigned int j = 0; j < m_msgPersonTrajectory.trajectory[i].fCenterX.size(); j++)
    {
      // this cluster
      uint64_t this_sTime = m_msgPersonTrajectory.trajectory[i].tStartTime[j].toNSec();
      uint64_t this_eTime = m_msgPersonTrajectory.trajectory[i].tEndTime[j].toNSec();
      float this_aveX = m_msgPersonTrajectory.trajectory[i].fCenterX[j];
      float this_aveY = m_msgPersonTrajectory.trajectory[i].fCenterY[j];

      // check top of each trajectory
      if (j == 0)
      {
        if (i == 0)
        {
          HumanBehavior tempHumanBehavior;
          human.behaviorWalking = BEHAVIOR_LOST;
          tempHumanBehavior.behavior = BEHAVIOR_LOST;
          tempHumanBehavior.sTime = getTime(GLOBAL_START_TIME.toNSec());
          tempHumanBehavior.eTime = getTime(this_sTime);
          tempHumanBehavior.fCenterX.push_back((float)EntrancePointX);
          tempHumanBehavior.fCenterY.push_back((float)EntrancePointY);
          walkingHistory.push_back(tempHumanBehavior);
        }
        else if (i > 0)
        {
          // recognize lost state from previous trajectory bottom to present trajectory top
          // time
          uint64_t previous_eTime = m_msgPersonTrajectory.trajectory[i - 1].tEndTime.back().toNSec();
          uint64_t present_sTime = m_msgPersonTrajectory.trajectory[i].tStartTime[j].toNSec();

          // middle of pos
          float middleX =
              (m_msgPersonTrajectory.trajectory[i].fCenterX[j] + m_msgPersonTrajectory.trajectory[i].fCenterX.back()) /
              2.0;
          float middleY =
              (m_msgPersonTrajectory.trajectory[i].fCenterY[j] + m_msgPersonTrajectory.trajectory[i].fCenterY.back()) /
              2.0;

          HumanBehavior tempHumanBehavior;
          human.behaviorWalking = BEHAVIOR_LOST;
          tempHumanBehavior.behavior = BEHAVIOR_LOST;
          tempHumanBehavior.sTime = getTime(previous_eTime);
          tempHumanBehavior.eTime = getTime(present_sTime);
          tempHumanBehavior.fCenterX.push_back(middleX);
          tempHumanBehavior.fCenterY.push_back(middleY);
          walkingHistory.push_back(tempHumanBehavior);
        }
      }

      unsigned int k = j + 2;  // estimate by gaitCycle
      if (k < m_msgPersonTrajectory.trajectory[i].fCenterX.size())
      {
        // next same cluster
        uint64_t next_sTime = m_msgPersonTrajectory.trajectory[i].tStartTime[k].toNSec();
        float next_aveX = m_msgPersonTrajectory.trajectory[i].fCenterX[k];
        float next_aveY = m_msgPersonTrajectory.trajectory[i].fCenterY[k];

        double gaitCycle = (double)(next_sTime - this_sTime) / 1.0e9;
        if (threshold <= gaitCycle)
        {
          if (human.behaviorWalking != BEHAVIOR_STAY_F)
          {
            HumanBehavior tempHumanBehavior;
            human.behaviorWalking = BEHAVIOR_STAY_F;
            tempHumanBehavior.behavior = BEHAVIOR_STAY_F;
            tempHumanBehavior.sTime = getTime(this_sTime);
            tempHumanBehavior.eTime = getTime(next_sTime);
            tempHumanBehavior.fCenterX.push_back(this_aveX);
            tempHumanBehavior.fCenterY.push_back(this_aveY);
            walkingHistory.push_back(tempHumanBehavior);
          }
          else
          {
            walkingHistory.back().fCenterX.push_back(next_aveX);
            walkingHistory.back().fCenterY.push_back(next_aveY);
            walkingHistory.back().eTime = getTime(next_sTime);
          }
        }
        else
        {
          if (human.behaviorWalking != BEHAVIOR_WALKING)
          {
            HumanBehavior tempHumanBehavior;
            human.behaviorWalking = BEHAVIOR_WALKING;
            tempHumanBehavior.behavior = BEHAVIOR_WALKING;
            tempHumanBehavior.sTime = getTime(this_sTime);
            tempHumanBehavior.eTime = getTime(next_sTime);  // TODO check correctly
            tempHumanBehavior.fCenterX.push_back(this_aveX);
            tempHumanBehavior.fCenterY.push_back(this_aveY);
            walkingHistory.push_back(tempHumanBehavior);
          }
          else
          {
            walkingHistory.back().fCenterX.push_back(this_aveX);
            walkingHistory.back().fCenterY.push_back(this_aveY);  // TODO check correctly
            walkingHistory.back().eTime = getTime(next_sTime);
          }
        }
      }
      else
      {
        walkingHistory.back().fCenterX.push_back(this_aveX);
        walkingHistory.back().fCenterY.push_back(this_aveY);
        walkingHistory.back().eTime = getTime(this_eTime);
      }
    }
  }
}

void estimateSittingBehavior()
{
  uint64_t sTime, eTime;
  float centerX, centerY;
  double sensingTime;

  // TODO:
  // 毎回全クリアせずに、差分の分析のみ行う処理に変更する必要あり
  nearChairList.clear();
  sittingHistory.clear();

  //------------------------------------------------------------------------------
  // get nearChairList
  bool isNearFirst = true;
  for (unsigned int i = 0; i < m_msgDetectedClusterList.cluster.size(); i++)
  {
    // center, sensing time
    centerX = m_msgDetectedClusterList.cluster[i].fCenterX.back();
    centerY = m_msgDetectedClusterList.cluster[i].fCenterY.back();
    sTime = m_msgDetectedClusterList.cluster[i].tStartTime.back().toNSec();
    eTime = m_msgDetectedClusterList.cluster[i].tEndTime.back().toNSec();
    sensingTime = (double)(getTime(eTime) - getTime(sTime));

    // get chair's position at the time
    float chairX, chairY;

    bool isGetChairPos = false;
    isGetChairPos = getChairPos(m_msgDetectedClusterList.cluster[i].tStartTime.back(), chairX, chairY);
    if (isGetChairPos)
    {
      if (isNearChair(chairX, chairY, centerX, centerY))
      {
        if (isNearFirst)
        {
          SumSensingTime tempSumSensingTime;
          tempSumSensingTime.sTime = GLOBAL_START_TIME.toNSec();
          tempSumSensingTime.eTime = GLOBAL_START_TIME.toNSec();
          tempSumSensingTime.sensingTime = 0.0;
          tempSumSensingTime.fCenterX = 0.0;
          tempSumSensingTime.fCenterY = 0.0;
          nearChairList.push_back(tempSumSensingTime);
        }
        SumSensingTime tempSumSensingTime;
        tempSumSensingTime.sTime = sTime;
        tempSumSensingTime.eTime = eTime;
        tempSumSensingTime.sensingTime = 0.0;
        tempSumSensingTime.fCenterX = centerX;
        tempSumSensingTime.fCenterY = centerY;
        nearChairList.push_back(tempSumSensingTime);
        isNearFirst = false;
      }
      else
      {
        isNearFirst = true;
      }
    }
  }

  //------------------------------------------------------------------------------
  // make sittingHistory

  human.behaviorSitting = BEHAVIOR_LOST;
  for (unsigned int i = 0; i < nearChairList.size(); i++)
  {
    // center, time
    float centerX = nearChairList[i].fCenterX;
    float centerY = nearChairList[i].fCenterY;
    uint64_t sTime = nearChairList[i].sTime;
    uint64_t eTime = nearChairList[i].eTime;

    if (centerX == 0 && centerY == 0)
    {
      human.behaviorSitting = BEHAVIOR_LOST;
    }
    else
    {
      // whether cluster(FootPrint) is under the table
      if (1000 <= centerX && centerX <= 1750 && 1500 <= centerY && centerY <= 2250)
      {
        if (human.behaviorSitting != BEHAVIOR_SITTING_C)
        {
          HumanBehavior tempHumanBehavior;
          tempHumanBehavior.behavior = BEHAVIOR_SITTING_C;
          tempHumanBehavior.eTime = getTime(eTime);
          tempHumanBehavior.fCenterX.push_back(centerX);
          tempHumanBehavior.fCenterY.push_back(centerY);

          if (human.behaviorSitting == BEHAVIOR_STAY_C)
            tempHumanBehavior.sTime = sittingHistory.back().eTime;
          else
            tempHumanBehavior.sTime = getTime(sTime);

          human.behaviorSitting = BEHAVIOR_SITTING_C;
          sittingHistory.push_back(tempHumanBehavior);
        }
        else
        {
          sittingHistory.back().fCenterX.push_back(centerX);
          sittingHistory.back().fCenterY.push_back(centerY);
          sittingHistory.back().eTime = getTime(eTime);
        }
      }
      else
      {
        if (human.behaviorSitting != BEHAVIOR_STAY_C)
        {
          HumanBehavior tempHumanBehavior;
          tempHumanBehavior.behavior = BEHAVIOR_STAY_C;
          tempHumanBehavior.eTime = getTime(eTime);
          tempHumanBehavior.fCenterX.push_back(centerX);
          tempHumanBehavior.fCenterY.push_back(centerY);

          if (human.behaviorSitting == BEHAVIOR_SITTING_C)
            tempHumanBehavior.sTime = sittingHistory.back().eTime;
          else
            tempHumanBehavior.sTime = getTime(sTime);

          human.behaviorSitting = BEHAVIOR_STAY_C;
          sittingHistory.push_back(tempHumanBehavior);
        }
        else
        {
          sittingHistory.back().fCenterX.push_back(centerX);
          sittingHistory.back().fCenterY.push_back(centerY);
          sittingHistory.back().eTime = getTime(eTime);
        }
      }
    }
  }
}

void estimateSleepingBehavior()
{
  uint64_t sTime, eTime;
  float centerX, centerY;
  double sensingTime;

  // TODO:
  // 毎回全クリアせずに、差分の分析のみ行う処理に変更する必要あり
  nearBedList.clear();
  sleepingHistory.clear();

  //------------------------------------------------------------------------------
  // get nearBedList
  bool isNearFirst = true;
  for (unsigned int i = 0; i < m_msgDetectedClusterList.cluster.size(); i++)
  {
    // center, sensing time
    centerX = m_msgDetectedClusterList.cluster[i].fCenterX.back();
    centerY = m_msgDetectedClusterList.cluster[i].fCenterY.back();
    sTime = m_msgDetectedClusterList.cluster[i].tStartTime.back().toNSec();
    eTime = m_msgDetectedClusterList.cluster[i].tEndTime.back().toNSec();
    sensingTime = (double)(getTime(eTime) - getTime(sTime));

    // get Bed's position at the time
    float bedX, bedY;

    bool isGetBedPos = false;
    isGetBedPos = getBedPos(m_msgDetectedClusterList.cluster[i].tStartTime.back(), bedX, bedY);
    if (isGetBedPos)
    {
      if (isNearBed(bedX, bedY, centerX, centerY))
      {
        if (isNearFirst)
        {
          SumSensingTime tempSumSensingTime;
          tempSumSensingTime.sTime = GLOBAL_START_TIME.toNSec();
          tempSumSensingTime.eTime = GLOBAL_START_TIME.toNSec();
          tempSumSensingTime.sensingTime = 0.0;
          tempSumSensingTime.fCenterX = 0.0;
          tempSumSensingTime.fCenterY = 0.0;
          nearBedList.push_back(tempSumSensingTime);
        }
        SumSensingTime tempSumSensingTime;
        tempSumSensingTime.sTime = sTime;
        tempSumSensingTime.eTime = eTime;
        tempSumSensingTime.sensingTime = 0.0;
        tempSumSensingTime.fCenterX = centerX;
        tempSumSensingTime.fCenterY = centerY;
        nearBedList.push_back(tempSumSensingTime);
        isNearFirst = false;
      }
      else
      {
        isNearFirst = true;
      }
    }
  }

  //	//------------------------------------------------------------------------------
  //	// get nearBedList
  //	bool isNearFirst = true;
  //	for(unsigned int i=0 ; i<m_msgPersonTrajectory.trajectory.size() ; i++){
  //		for(unsigned int j=0 ; j<m_msgPersonTrajectory.trajectory[i].fCenterX.size() ; j++){
  //
  //			// center, sensing time
  //			centerX	= m_msgPersonTrajectory.trajectory[i].fCenterX[j];
  //			centerY	= m_msgPersonTrajectory.trajectory[i].fCenterY[j];
  //			sTime		= m_msgPersonTrajectory.trajectory[i].tStartTime[j].toNSec();
  //			eTime 		= m_msgPersonTrajectory.trajectory[i].tEndTime[j].toNSec();
  //			sensingTime = (double)(getTime(eTime)-getTime(sTime));
  //
  //			// get Bed's position at the time
  //			float bedX, bedY;
  //
  //			bool isGetBedPos = false;
  //			isGetBedPos = getBedPos(m_msgPersonTrajectory.trajectory[i].tStartTime[j], bedX, bedY);
  //			if(isGetBedPos){
  //				if(isNearBed(bedX, bedY, centerX, centerY)){
  //					if(isNearFirst){
  //						// set dummy element
  //						SumSensingTime tempSumSensingTime;
  //						tempSumSensingTime.sTime			= 0.0;
  //						tempSumSensingTime.eTime			= 0.0;
  //						tempSumSensingTime.sensingTime	= 0.0;
  //						tempSumSensingTime.fCenterX		= 0.0;
  //						tempSumSensingTime.fCenterY		= 0.0;
  //						nearBedList.push_back(tempSumSensingTime);
  //					}
  //					SumSensingTime tempSumSensingTime;
  //					tempSumSensingTime.sTime			= sTime;
  //					tempSumSensingTime.eTime			= eTime;
  //					tempSumSensingTime.sensingTime	= 0.0;
  //					tempSumSensingTime.fCenterX		= centerX;
  //					tempSumSensingTime.fCenterY		= centerY;
  //					nearBedList.push_back(tempSumSensingTime);
  //					isNearFirst = false;
  //				}
  //				else{
  //					isNearFirst = true;
  //				}
  //			}
  //		}
  //	}

  //------------------------------------------------------------------------------
  // make sleepingHistory
  human.behaviorSleeping = BEHAVIOR_LOST;
  for (unsigned int i = 0; i < nearBedList.size(); i++)
  {
    // this_center, this_time
    float this_centerX = nearBedList[i].fCenterX;
    float this_centerY = nearBedList[i].fCenterY;
    uint64_t this_sTime = nearBedList[i].sTime;
    uint64_t this_eTime = nearBedList[i].eTime;

    if (this_centerX == 0 && this_centerY == 0)
    {
      human.behaviorSleeping = BEHAVIOR_LOST;
    }
    else
    {
      if (human.behaviorSleeping != BEHAVIOR_SITTING_B)
      {
        HumanBehavior tempHumanBehavior;
        human.behaviorSleeping = BEHAVIOR_SITTING_B;
        tempHumanBehavior.behavior = BEHAVIOR_SITTING_B;
        tempHumanBehavior.sTime = getTime(this_sTime);
        tempHumanBehavior.eTime = getTime(this_eTime);
        tempHumanBehavior.fCenterX.push_back(this_centerX);
        tempHumanBehavior.fCenterY.push_back(this_centerY);
        sleepingHistory.push_back(tempHumanBehavior);
      }
      else
      {
        sleepingHistory.back().fCenterX.push_back(this_centerX);
        sleepingHistory.back().fCenterY.push_back(this_centerY);
        sleepingHistory.back().eTime = getTime(this_eTime);
      }

      // check whether interval is long or not
      if (i + 1 < nearBedList.size())
      {
        // next_center, next_time
        float next_centerX = nearBedList[i + 1].fCenterX;
        float next_centerY = nearBedList[i + 1].fCenterY;
        uint64_t next_sTime = nearBedList[i + 1].sTime;
        uint64_t next_eTime = nearBedList[i + 1].eTime;

        if (getTime(this_eTime) + START_SLEEPING <= getTime(next_sTime))
        {
          HumanBehavior tempHumanBehavior;
          human.behaviorSleeping = BEHAVIOR_STAY_B;
          tempHumanBehavior.behavior = BEHAVIOR_STAY_B;
          tempHumanBehavior.sTime = getTime(this_eTime);
          tempHumanBehavior.eTime = getTime(next_sTime);
          tempHumanBehavior.fCenterX.push_back(3500);
          tempHumanBehavior.fCenterY.push_back(3500);
          sleepingHistory.push_back(tempHumanBehavior);
        }
      }
    }
  }
}

void makeBehaviorResult()
{
  std::vector< int > row;
  int maxTime = (int)getTime(GLOBAL_LAST_TIME.toNSec());

  behaviorResult.clear();

  // behaviorResultTableV
  // 4: number of estimation method(walking, sitting, sleeping, mergedResult)
  for (int i = 0; i < maxTime; i++)
  {
    row.clear();
    for (unsigned int j = 0; j < 4; j++)
    {
      row.push_back(0);
    }
    behaviorResult.push_back(row);
  }

  // estimation result by walking model
  for (unsigned int i = 0; i < walkingHistory.size(); i++)
  {
    int behavior;
    double sTime, eTime;

    behavior = walkingHistory[i].behavior;
    sTime = walkingHistory[i].sTime;
    eTime = walkingHistory[i].eTime;

    for (int t = (int)sTime; t < (int)eTime; t++)
    {
      behaviorResult[t][0] = behavior;
    }
  }

  // estimation result by sitting model
  for (unsigned int i = 0; i < sittingHistory.size(); i++)
  {
    int behavior;
    double sTime, eTime;
    behavior = sittingHistory[i].behavior;
    sTime = sittingHistory[i].sTime;
    eTime = sittingHistory[i].eTime;

    for (int t = (int)sTime; t < (int)eTime; t++)
    {
      behaviorResult[t][1] = behavior;
    }
  }

  // estimation result by sleeping model
  for (unsigned int i = 0; i < sleepingHistory.size(); i++)
  {
    int behavior;
    double sTime, eTime;

    behavior = sleepingHistory[i].behavior;
    sTime = sleepingHistory[i].sTime;
    eTime = sleepingHistory[i].eTime;

    for (int t = (int)sTime; t < (int)eTime; t++)
    {
      behaviorResult[t][2] = behavior;
    }
  }

  // merged behaviorState by 3 model results
  mergeBehavior();

  // Publish message
  tms_msg_ss::bas_behavior_data bas_behavior_data;
  bas_behavior_data.header.frame_id = "bas_behavior_data";
  bas_behavior_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  bas_behavior_data.tMeasuredTime = ros::Time::now() + ros::Duration(9 * 60 * 60);
  for (unsigned int t = 0; t < behaviorResult.size(); t++)
  {
    ros::Time timeStamp = GLOBAL_START_TIME + ros::Duration(t);
    bas_behavior_data.tTimeStamp.push_back(timeStamp);
    bas_behavior_data.iBehaviorWalking.push_back(behaviorResult[t][0]);
    bas_behavior_data.iBehaviorSitting.push_back(behaviorResult[t][1]);
    bas_behavior_data.iBehaviorSleeping.push_back(behaviorResult[t][2]);
    bas_behavior_data.iBehaviorMerged.push_back(behaviorResult[t][3]);
  }
  rosPubHumanBehavior.publish(bas_behavior_data);
}

void mergeBehavior()
{
  int initialState = 0;
  int preState;
  int mergedBehavior;

  for (int t = 0; t < behaviorResult.size(); t++)
  {
    if (t == 0)
      preState = initialState;
    else
      preState = behaviorResult[t - 1][3];

    mergedBehavior = getEstimatedState(t, preState, behaviorResult[t][0], behaviorResult[t][1], behaviorResult[t][2]);
    behaviorResult[t][3] = mergedBehavior;
  }
}

bool isExit(int t)
{
  bool isExit = false;

  for (unsigned int i = 0; i < walkingHistory.size(); i++)
  {
    if (int(walkingHistory[i].sTime) <= t && t <= int(walkingHistory[i].eTime))
    {
      float fCenterX = walkingHistory[i].fCenterX.back();
      float fCenterY = walkingHistory[i].fCenterY.back();

      // check whether in entrance or not
      if ((0 <= fCenterX && fCenterX <= 2500) && (0 <= fCenterY && fCenterY <= 500))
      {
        isExit = true;
        break;
      }
    }
  }
  return isExit;
}

int getEstimatedState(int t, int preState, int walkingBehavior, int sittingBehavior, int sleepingBehavior)
{
  int mergedBehavior;

  bool isPrint;
  if (walkingBehavior == 2 && sittingBehavior == 0 && sleepingBehavior == 5)
    isPrint = true;
  else
    isPrint = false;

  switch (preState)
  {
    //------------------------------------------------------------------------------
    // 事前状態：退室中
    case BEHAVIOR_LOST:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_LOST;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：歩行中
    case BEHAVIOR_WALKING:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：フロア内静止中
    case BEHAVIOR_STAY_F:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  if (isExit(t))
                    mergedBehavior = BEHAVIOR_LOST;
                  else
                    mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：椅子付近滞在中
    case BEHAVIOR_STAY_C:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：椅子着座中
    case BEHAVIOR_SITTING_C:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_C;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_SITTING_C;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：ベッド腰掛け
    case BEHAVIOR_SITTING_B:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
      }
      break;
    //------------------------------------------------------------------------------
    // 事前状態：ベッド上滞在
    case BEHAVIOR_STAY_B:
      switch (walkingBehavior)
      {
        case BEHAVIOR_LOST:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_WALKING:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_WALKING;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
        case BEHAVIOR_STAY_F:
          switch (sittingBehavior)
          {
            case BEHAVIOR_LOST:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_STAY_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
            case BEHAVIOR_SITTING_C:
              switch (sleepingBehavior)
              {
                case BEHAVIOR_LOST:
                  mergedBehavior = BEHAVIOR_STAY_F;
                  break;
                case BEHAVIOR_SITTING_B:
                  mergedBehavior = BEHAVIOR_SITTING_B;
                  break;
                case BEHAVIOR_STAY_B:
                  mergedBehavior = BEHAVIOR_STAY_B;
                  break;
              }
              break;
          }
          break;
      }
      break;
  }

  return mergedBehavior;
}

bool getBedPos(ros::Time time, float &bedX, float &bedY)
{
  bool isGetBedPos = false;

  // Now bed's position is stable.
  isGetBedPos = true;
  bedX = 3500;
  bedY = 3500;

  return isGetBedPos;
}

bool isNearBed(float bedX, float bedY, float centerX, float centerY)
{
  bool isNearBed;

  // Now bed's position is stable.
  if (bedX - 1500 <= centerX && centerX <= bedX + 1500 && bedY - 1000 <= centerY && centerY <= bedY + 1000)
  {
    isNearBed = true;
  }
  else
  {
    isNearBed = false;
  }
  return isNearBed;
}

bool getChairPos(ros::Time time, float &chairX, float &chairY)
{
  bool isGetChairPos = false;

  for (unsigned int i = 0; i < m_msgChairBuffer.tStartTime.size(); i++)
  {
    if (m_msgChairBuffer.tStartTime[i] <= time && time <= m_msgChairBuffer.tEndTime[i])
    {
      isGetChairPos = true;
      chairX = m_msgChairBuffer.fCenterX[i];
      chairY = m_msgChairBuffer.fCenterY[i];
      break;
    }
  }
  if (!isGetChairPos)
  {
    chairX = chairY = 0;
  }

  return isGetChairPos;
}

bool isNearChair(float chairX, float chairY, float centerX, float centerY)
{
  bool isNearChair;
  float len;

  len = getLength(chairX, chairY, centerX, centerY);
  if (len < AREA_CHAIR)
    isNearChair = true;
  else
    isNearChair = false;

  return isNearChair;
}

void displayWalkingHistory()
{
  printf("\n-----------------------------------------\n");
  for (unsigned int i = 0; i < walkingHistory.size(); i++)
  {
    printf("%lf\t%lf:\t%d\t%f\t%f\n", walkingHistory[i].sTime, walkingHistory[i].eTime, walkingHistory[i].behavior,
           walkingHistory[i].fCenterX.back(), walkingHistory[i].fCenterY.back());
  }
}

void displaySleepingHistory()
{
  printf("\n-----------------------------------------\n");
  for (unsigned int i = 0; i < sleepingHistory.size(); i++)
  {
    printf("\n---------------\n");
    printf("%lf\t%lf:\t%d\n", sleepingHistory[i].sTime, sleepingHistory[i].eTime, sleepingHistory[i].behavior);
    for (unsigned int j = 0; j < sleepingHistory[i].fCenterX.size(); j++)
    {
      printf("%f\t%f\n", sleepingHistory[i].fCenterX[j], sleepingHistory[i].fCenterY[j]);
    }
  }
}

void outputLogfile()
{
  //------------------------------------------------------------------------------
  // nearChairData
  outputNearChairData();
  outputNearBedData();
  outputGaitCycleData();
  outputStepCycleData();
  outputHumanBehaviorData();
  outputFootPrintListData();
  outputFootPrintAllData();
  outputDetectedClusterListData();
}

void outputNearChairData()
{
  // chair buffer
  ofs1 << std::endl;
  ofs1 << "Chair Buffer" << std::endl;
  ofs1 << "sTime"
       << "\t"
       << "eTime"
       << "\t"
       << "fCenterX"
       << "\t"
       << "fCenterY" << std::endl;
  for (unsigned int i = 0; i < m_msgChairBuffer.tStartTime.size(); i++)
  {
    uint64_t sTime = m_msgChairBuffer.tStartTime[i].toNSec();
    uint64_t eTime = m_msgChairBuffer.tEndTime[i].toNSec();
    float fCenterX = m_msgChairBuffer.fCenterX[i];
    float fCenterY = m_msgChairBuffer.fCenterY[i];

    ofs1 << getTime(sTime) << "\t" << getTime(eTime) << "\t" << fCenterX << "\t" << fCenterY;
    ofs1 << std::endl;
  }
  ofs1 << std::endl;

  // nearChairList
  ofs1 << "Near chair data" << std::endl;
  ofs1 << "sTime"
       << "\t"
       << "eTime"
       << "\t"
       << "sensingTime"
       << "\t"
       << "fCenterX"
       << "\t"
       << "fCenterY" << std::endl;

  for (unsigned int i = 0; i < nearChairList.size(); i++)
  {
    uint64_t sTime = nearChairList[i].sTime;
    uint64_t eTime = nearChairList[i].eTime;
    double sensingTime = nearChairList[i].sensingTime;
    float fCenterX = nearChairList[i].fCenterX;
    float fCenterY = nearChairList[i].fCenterY;
    if (sTime == 0.0 && eTime == 0.0)
      ofs1 << 0 << "\t" << 0 << "\t" << sensingTime << "\t" << fCenterX << "\t" << fCenterY;
    else
      ofs1 << getTime(sTime) << "\t" << getTime(eTime) << "\t" << sensingTime << "\t" << fCenterX << "\t" << fCenterY;
    ofs1 << std::endl;
  }
  ofs1 << std::endl;
}

void outputNearBedData()
{
  ofs2 << "Near bed data" << std::endl;
  ofs2 << "sTime"
       << "\t"
       << "eTime"
       << "\t"
       << "sensingTime" << std::endl;

  for (unsigned int i = 0; i < nearBedList.size(); i++)
  {
    uint64_t sTime = nearBedList[i].sTime;
    uint64_t eTime = nearBedList[i].eTime;
    double sensingTime = nearBedList[i].sensingTime;

    if (sTime == 0.0 && eTime == 0.0)
      ofs2 << 0 << "\t" << 0 << "\t" << sensingTime;
    else
      ofs2 << getTime(sTime) << "\t" << getTime(eTime) << "\t" << sensingTime;
    ofs2 << std::endl;
  }
  ofs2 << std::endl;
}

void outputGaitCycleData()
{
  if (WALKINGMODEL_METHOD == 1)
  {
    //------------------------------------------------------------------------------
    // gaitCycle
    for (unsigned int i = 0; i < human.gaitCycleData.size(); i++)
    {
      ofs3 << "---------------------------------------" << std::endl;
      ofs3 << "humanTrajectory[" << i << "]" << std::endl;
      ofs3 << "---------------------------------------" << std::endl;
      ofs3 << "gaitCycle"
           << "\t"
           << "time[sec]"
           << "\t"
           << "isValid"
           << "\t"
           << "μ+2σ" << std::endl;
      for (unsigned int j = 0; j < human.gaitCycleData[i].size(); j++)
      {
        ofs3 << "[" << j + 1 << "]-[" << j + 3 << "]"
             << "\t";
        ofs3 << human.gaitCycleData[i][j].value;
        if (!human.gaitCycleData[i][j].isValid)
          ofs3 << "\t"
               << "false";
        else
          ofs3 << "\t";
        ofs3 << "\t" << human.baseGaitCycle[i] + 2 * sqrt(human.baseGaitCycleVar[i]);
        ofs3 << std::endl;
      }
      ofs3 << "----------" << std::endl;
      ofs3 << "gaitCycle Average"
           << "\t" << human.baseGaitCycle[i] << std::endl;
      ofs3 << "gaitCycle Var"
           << "\t" << human.baseGaitCycleVar[i] << std::endl;
      ofs3 << "gaitCycle Sigma"
           << "\t" << sqrt(human.baseGaitCycleVar[i]) << std::endl;
      ofs3 << std::endl;
    }

    // gaitCycleByAll
    ofs3 << "---------------------------------------" << std::endl;
    ofs3 << "humanTrajectory all data" << std::endl;
    ofs3 << "---------------------------------------" << std::endl;
    ofs3 << "gaitCycle"
         << "\t"
         << "time[sec]"
         << "\t"
         << "isValid"
         << "\t"
         << "μ+2σ" << std::endl;
    for (unsigned int i = 0; i < human.gaitCycleDataByAll.size(); i++)
    {
      ofs3 << "[" << i + 1 << "]-[" << i + 3 << "]"
           << "\t";
      ofs3 << human.gaitCycleDataByAll[i].value;
      if (!human.gaitCycleDataByAll[i].isValid)
        ofs3 << "\t"
             << "false";
      else
        ofs3 << "\t";

      ofs3 << "\t" << human.baseGaitCycleByAll + 2 * sqrt(human.baseGaitCycleVarByAll);
      ofs3 << std::endl;
    }
    ofs3 << "----------" << std::endl;
    ofs3 << "gaitCycle Average"
         << "\t" << human.baseGaitCycleByAll << std::endl;
    ofs3 << "gaitCycle Var"
         << "\t" << human.baseGaitCycleVarByAll << std::endl;
    ofs3 << "gaitCycle Sigma"
         << "\t" << sqrt(human.baseGaitCycleVarByAll) << std::endl;
    ofs3 << std::endl;
  }
}

void outputStepCycleData()
{
  if (WALKINGMODEL_METHOD == 1)
  {
    for (unsigned int i = 0; i < human.stepCycleData.size(); i++)
    {
      ofs3 << "---------------------------------------" << std::endl;
      ofs3 << "humanTrajectory[" << i << "]" << std::endl;
      ofs3 << "---------------------------------------" << std::endl;
      ofs3 << "stepCycle"
           << "\t"
           << "time[sec]"
           << "\t"
           << "isValid" << std::endl;
      for (unsigned int j = 0; j < human.stepCycleData[i].size(); j++)
      {
        ofs3 << "[" << j + 1 << "]-[" << j + 2 << "]"
             << "\t";
        ofs3 << human.stepCycleData[i][j].value;
        if (!human.stepCycleData[i][j].isValid)
          ofs3 << "\t"
               << "false";
        ofs3 << std::endl;
      }
      ofs3 << "----------" << std::endl;
      ofs3 << "stepCycle Average"
           << "\t" << human.baseStepCycle[i] << std::endl;
      ofs3 << "stepCycle Var"
           << "\t" << human.baseStepCycleVar[i] << std::endl;
      ofs3 << "stepCycle Sigma"
           << "\t" << sqrt(human.baseStepCycleVar[i]) << std::endl;
      ofs3 << std::endl;
    }

    // stepCycleByAll
    ofs3 << "---------------------------------------" << std::endl;
    ofs3 << "humanTrajectory all data" << std::endl;
    ofs3 << "---------------------------------------" << std::endl;
    ofs3 << "stepCycle"
         << "\t"
         << "time[sec]"
         << "\t"
         << "isValid" << std::endl;
    for (unsigned int i = 0; i < human.stepCycleDataByAll.size(); i++)
    {
      ofs3 << "[" << i + 1 << "]-[" << i + 2 << "]"
           << "\t";
      ofs3 << human.stepCycleDataByAll[i].value;
      if (!human.stepCycleDataByAll[i].isValid)
        ofs3 << "\t"
             << "false";
      ofs3 << std::endl;
    }
    ofs3 << "----------" << std::endl;
    ofs3 << "stepCycle Average"
         << "\t" << human.baseStepCycleByAll << std::endl;
    ofs3 << "stepCycle Var"
         << "\t" << human.baseStepCycleVarByAll << std::endl;
    ofs3 << "stepCycle Sigma"
         << "\t" << sqrt(human.baseStepCycleVarByAll) << std::endl;
    ofs3 << std::endl;
  }
}

void outputHumanBehaviorData()
{
  ofs4 << std::endl;
  ofs4 << "BehaviorResult" << std::endl;
  ofs4 << "-------------" << std::endl;

  for (unsigned int i = 0; i < behaviorResult.size(); i++)
  {
    for (unsigned int j = 0; j < behaviorResult[i].size(); j++)
    {
      ofs4 << behaviorResult[i][j] << "\t";
    }
    ofs4 << std::endl;
  }
  ofs4 << "-------------" << std::endl;
  ofs4 << std::endl;
}

void outputFootPrintListData()
{
  //------------------------------------------------------------------------------
  // personTrajectory
  ofs5 << "personTrajectory" << std::endl;
  ofs5 << "no"
       << "\t"
       << "sTime"
       << "\t"
       << "eTime"
       << "\t"
       << "sensingTime"
       << "\t"
       << "fCenterX"
       << "\t"
       << "fCenterY" << std::endl;

  for (unsigned int i = 0; i < m_msgPersonTrajectory.trajectory.size(); i++)
  {
    for (unsigned int j = 0; j < m_msgPersonTrajectory.trajectory[i].fCenterX.size(); j++)
    {
      uint64_t sTime = m_msgPersonTrajectory.trajectory[i].tStartTime[j].toNSec();
      uint64_t eTime = m_msgPersonTrajectory.trajectory[i].tEndTime[j].toNSec();
      double sensingTime = getTime(eTime) - getTime(sTime);
      float fCenterX = m_msgPersonTrajectory.trajectory[i].fCenterX[j];
      float fCenterY = m_msgPersonTrajectory.trajectory[i].fCenterY[j];

      int index;
      if (i == 0)
      {
        index = j + 1;
      }
      else
      {
        int count = 0;
        for (unsigned int k = 0; k < i; k++)
        {
          count += m_msgPersonTrajectory.trajectory[k].fCenterX.size();
        }
        index = count + j + 1;
      }
      ofs5 << index << "\t" << getTime(sTime) << "\t" << getTime(eTime) << "\t" << sensingTime << "\t" << fCenterX
           << "\t" << fCenterY << std::endl;
    }
    ofs5 << std::endl;
  }
  ofs5 << std::endl;
}

void outputFootPrintAllData()
{
}

void outputDetectedClusterListData()
{
  ofs7 << "sTime"
       << "\t"
       << "eTime"
       << "\t"
       << "sensingTime"
       << "\t"
       << "fCenterX"
       << "\t"
       << "fCenterY" << std::endl;
  for (unsigned int i = 0; i < m_msgDetectedClusterList.cluster.size(); i++)
  {
    tms_msg_ss::fss_observed_datas cluster;
    cluster = m_msgDetectedClusterList.cluster[i];

    uint64_t sTime = cluster.tStartTime.back().toNSec();
    uint64_t eTime = cluster.tEndTime.back().toNSec();
    double sensingTime = getTime(eTime) - getTime(sTime);
    float fCenterX = cluster.fCenterX.back();
    float fCenterY = cluster.fCenterY.back();

    ofs7 << getTime(sTime) << "\t" << getTime(eTime) << "\t" << sensingTime << "\t" << fCenterX << "\t" << fCenterY
         << std::endl;
  }
  ofs7 << std::endl;
}
