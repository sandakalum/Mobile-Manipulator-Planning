//------------------------------------------------------------------------------
// @file   : fss_person_trakcing.cpp
// @brief  : find footprint cluster by classify info and personTrajectory
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.11.01)
// @date   : 2012.11.01
//------------------------------------------------------------------------------

#include <time.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <list>
#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_ss/fss_tf_data.h>
#include <tms_msg_ss/fss_tf_datas.h>
#include <tms_msg_ss/fss_person_trajectory_data.h>
#include <tms_msg_ss/fss_detected_cluster_data.h>
#include <tms_msg_ss/fss_observed_datas.h>
#include <tms_ss_fss_exe/common.h>
#include <tms_ss_fss_exe/Cluster.h>
#include <fstream>
#include <sstream>

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

//------------------------------------------------------------------------------
// declaration of functions

int getNewClusterID();
bool mergeInterruptedCluster();
void makeTrajectoryTree();
void getHumanTrajectory();
void sendPersonData();
void sendDetectedCluster();
void insertDetectedClusterList();
void callback(const tms_msg_ss::fss_class_data::ConstPtr& msg);
float getProbabilityFootsize(float fSize);
float getMaxCost(std::list< Node >::iterator it);
float getProbabilitySensingtime(double sensingTime);
float getLength(float x1, float y1, float x2, float y2);
float getScore(std::list< Cluster >::iterator it, const tms_msg_ss::fss_class_data::ConstPtr& msg, int gt_index);
double getTime(uint64_t time);

void displayCompareList();
void displayDetectedClusterList();
void displayTrajectoryTree();
void displayHumanTrajectory();

void outputTrajectoryTreeData();

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

//------------------------------------------------------------------------------
// output stream
time_t now;
std::ofstream ofs1;
char filename1[256];  // for trajectoryTree

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  if (isOutput)
  {
    now = time(NULL);
    sprintf(filename1, "TrajectoryTreeData_%ld.txt", now);
    ofs1.open(filename1);
  }

  ros::init(argc, argv, "fss_person_tracking");
  ros::NodeHandle nh;

  // rosSub					= nh.subscribe("fss_class_data", 10, callback);
  rosSub = nh.subscribe("fss_unknown_class_data", 10, callback);
  rosPersonTrajectory = nh.advertise< tms_msg_ss::fss_person_trajectory_data >("fss_person_trajectory_data", 10);
  rosDetectedCluster = nh.advertise< tms_msg_ss::fss_detected_cluster_data >("fss_detected_cluster_data", 10);
  ros::spin();

  //------------------------------------------------------------------------------
  // for output
  if (isOutput)
  {
    printf("------\n");
    printf("Output files.\n");
    outputTrajectoryTreeData();
    ofs1.close();
    printf("-%s\n", filename1);
  }

  return (0);
}

//------------------------------------------------------------------------------
// Definition of Functions
void callback(const tms_msg_ss::fss_class_data::ConstPtr& msg)
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

  //----------------------------------------------------------------------------
  // insert cluster into detectedClusterList from preDetectedClusterList
  if (preDetectedClusterList.size() > 0)
  {
    if (isMerge && detectedClusterList.size() > 0)
    {
      bool isMerged = true;
      while (isMerged)
      {
        isMerged = mergeInterruptedCluster();
      }
    }
    insertDetectedClusterList();
  }
  sendDetectedCluster();

  printf("detectedClusterList: %ld\n", detectedClusterList.size());
  printf("humanTrajectory: %ld\n", humanTrajectory.size());

  makeTrajectoryTree();
  getHumanTrajectory();

  // displayDetectedClusterList();
  // displayTrajectoryTree();
  // displayHumanTrajectory();
  sendPersonData();
}

int getNewClusterID()
{
  detectedClusterIDMax++;
  return detectedClusterIDMax;
}

float getLength(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float getProbabilityFootsize(float fSize)
{
  if (FOOTPRINTSIZE_MIN < fSize && fSize < FOOTPRINTSIZE_MAX)
    return 0.8;
  else
    return 0.2;
}

float getProbabilitySensingtime(double sensingTime)
{
  if (SENSINGTIME_MIN < sensingTime)  //&& span < SENSINGTIME_MAX)
    return 0.8;
  else
    return 0.2;
}

float getScore(std::list< Cluster >::iterator it, const tms_msg_ss::fss_class_data::ConstPtr& msg, int gt_index)
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

bool mergeInterruptedCluster()
{
  bool isMerged = false;

  uint64_t this_sTime, this_eTime;
  uint64_t next_sTime, next_eTime;
  float this_centerX, this_centerY;
  float next_centerX, next_centerY;

  std::list< Cluster >::iterator this_it, next_it;
  for (next_it = preDetectedClusterList.begin(); next_it != preDetectedClusterList.end();)
  {
    // center
    next_centerX = next_it->fAveCenterX();
    next_centerY = next_it->fAveCenterY();

    // sensing time
    next_sTime = next_it->tMeasuredTime.front().toNSec();
    next_eTime = next_it->tMeasuredTime.back().toNSec();

    bool isThisMerged = false;
    for (this_it = detectedClusterList.begin(); this_it != detectedClusterList.end(); this_it++)
    {
      // center
      this_centerX = this_it->fAveCenterX();
      this_centerY = this_it->fAveCenterY();

      // sensing time
      this_sTime = this_it->tMeasuredTime.front().toNSec();
      this_eTime = this_it->tMeasuredTime.back().toNSec();

      float len = getLength(this_centerX, this_centerY, next_centerX, next_centerY);
      if (len < SAMECLUSTER)
      {
        if (getTime(next_sTime) < getTime(this_eTime) + (double)OVERLAPTIME_ROOM)
        {
          // merged (update eTime, sensingTime in detectedClusterList)
          isMerged = isThisMerged = true;
          if (this_eTime < next_eTime)
          {
            this_it->sensingTime = getTime(next_eTime) - getTime(this_sTime);
            this_it->tMeasuredTime.push_back(next_it->tMeasuredTime.back());
          }
          next_it = preDetectedClusterList.erase(next_it);
          break;
        }
      }
    }
    if (!isThisMerged)
      next_it++;
  }
  return isMerged;
}

void insertDetectedClusterList()
{
  if (detectedClusterList.size() == 0)
  {
    std::list< Cluster >::iterator d_it;
    for (d_it = preDetectedClusterList.begin(); d_it != preDetectedClusterList.end();)
    {
      Cluster tempCluster;
      tempCluster.iID = getNewClusterID();
      tempCluster.sensingTime = d_it->sensingTime;
      tempCluster.tMeasuredTime = d_it->tMeasuredTime;
      tempCluster.fCenterX = d_it->fCenterX;
      tempCluster.fCenterY = d_it->fCenterY;

      tempCluster.iNumPoint = d_it->iNumPoint;
      tempCluster.fSize = d_it->fSize;
      tempCluster.fSizeMax = d_it->fSizeMax;
      tempCluster.LrfData = d_it->LrfData;

      tempCluster.fMiddleX = d_it->fMiddleX;
      tempCluster.fMiddleY = d_it->fMiddleY;

      tempCluster.fRectangleMinX = d_it->fRectangleMinX;
      tempCluster.fRectangleMinY = d_it->fRectangleMinY;
      tempCluster.fRectangleMaxX = d_it->fRectangleMaxX;
      tempCluster.fRectangleMaxY = d_it->fRectangleMaxY;

      detectedClusterList.push_back(tempCluster);
      d_it = preDetectedClusterList.erase(d_it);
    }
  }
  else
  {
    // insert cluster to detectedClusterList order by start start sensingTime
    std::list< Cluster >::iterator d_it;
    for (d_it = preDetectedClusterList.begin(); d_it != preDetectedClusterList.end();)
    {
      uint64_t sTime = d_it->tMeasuredTime.front().toNSec();

      std::list< Cluster >::iterator p_it;
      p_it = detectedClusterList.end();
      for (unsigned int i = 0; i < detectedClusterList.size(); i++)
      {
        p_it--;

        uint64_t _sTime = p_it->tMeasuredTime.front().toNSec();
        if (sTime >= _sTime)
        {
          p_it++;
          break;
        }
      }

      Cluster tempCluster;
      tempCluster.iID = getNewClusterID();
      tempCluster.sensingTime = d_it->sensingTime;
      tempCluster.tMeasuredTime = d_it->tMeasuredTime;
      tempCluster.fCenterX = d_it->fCenterX;
      tempCluster.fCenterY = d_it->fCenterY;
      tempCluster.iNumPoint = d_it->iNumPoint;
      tempCluster.fSize = d_it->fSize;
      tempCluster.fSizeMax = d_it->fSizeMax;
      tempCluster.LrfData = d_it->LrfData;

      tempCluster.fMiddleX = d_it->fMiddleX;
      tempCluster.fMiddleY = d_it->fMiddleY;

      tempCluster.fRectangleMinX = d_it->fRectangleMinX;
      tempCluster.fRectangleMinY = d_it->fRectangleMinY;
      tempCluster.fRectangleMaxX = d_it->fRectangleMaxX;
      tempCluster.fRectangleMaxY = d_it->fRectangleMaxY;

      p_it = detectedClusterList.insert(p_it, tempCluster);
      d_it = preDetectedClusterList.erase(d_it);
    }
  }
}

void makeTrajectoryTree()
{
  uint64_t detected_sTime, detected_eTime;
  uint64_t sensing_sTime, sensing_eTime;
  float detected_centerX, detected_centerY;
  float sensing_centerX, sensing_centerY;

  //------------------------------------------------------------------------------
  // make each node of detectedClusters at first
  std::list< Cluster >::iterator detected_it;
  for (detected_it = detectedClusterList.begin(); detected_it != detectedClusterList.end(); detected_it++)
  {
    // trajectoryTree内に既にノード化しているかチェック
    bool isCreateNode = false;
    std::list< Node >::iterator this_it;
    for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
    {
      if (detected_it->iID == this_it->index)
      {
        isCreateNode = true;
        break;
      }
    }
    // ノード化していないクラスタのみノード化する
    if (!isCreateNode)
    {
      // 計測中のクラスタの方が計測開始時間が早い場合(case1)や、併合される可能性がある場合(case2)はノードを作成しない
      bool isProbability = false;
      std::list< Cluster >::iterator sensing_it;
      for (sensing_it = sensingClusterList.begin(); sensing_it != sensingClusterList.end(); sensing_it++)
      {
        // center, sensing time
        sensing_centerX = sensing_it->fAveCenterX();
        sensing_centerY = sensing_it->fAveCenterY();
        sensing_sTime = sensing_it->tMeasuredTime.front().toNSec();
        sensing_eTime = sensing_it->tMeasuredTime.back().toNSec();

        // case1's check
        if (sensing_sTime < detected_sTime)
        {
          isProbability = true;
          break;
        }
        // case2's check
        float len = getLength(detected_centerX, detected_centerY, sensing_centerX, sensing_centerY);
        if (len < SAMECLUSTER)
        {
          if (getTime(sensing_sTime) < getTime(detected_eTime) + (double)OVERLAPTIME_ROOM)
          {
            isProbability = true;
            break;
          }
        }
      }

      if (isProbability)
        break;
      else
      {
        Node tempNode;
        tempNode.index = detected_it->iID;
        tempNode.cluster = *detected_it;
        tempNode.cost = -1;
        tempNode.isLeaf = true;
        trajectoryTree.push_back(tempNode);
        trajectoryTreeClusterIDMax = detected_it->iID;
      }
    }
  }
}

void getHumanTrajectory()
{
  int this_index, child_index;
  std::list< Node >::iterator this_it, child_it;
  uint64_t this_sTime, this_eTime;
  uint64_t child_sTime, child_eTime;

  float this_centerX, this_centerY;
  float child_centerX, child_centerY;

  //------------------------------------------------------------------------------
  // get links among nodes
  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    // center, sensing time
    this_centerX = this_it->cluster.fCenterX.back();  // backX of centers
    this_centerY = this_it->cluster.fCenterY.back();  // backY of centers
    this_sTime = this_it->cluster.tMeasuredTime.front().toNSec();
    this_eTime = this_it->cluster.tMeasuredTime.back().toNSec();

    this_index = this_it->index;
    child_it = this_it;
    child_it++;

    for (; child_it != trajectoryTree.end(); child_it++)
    {
      bool isLinked = false;
      for (unsigned int i = 0; i < this_it->child_index.size(); i++)
      {
        if (child_it->index == this_it->child_index[i])
        {
          isLinked = true;
          break;
        }
      }
      if (!isLinked)
      {
        // center, sensing time
        child_centerX = child_it->cluster.fCenterX.front();  // frontX of centers
        child_centerY = child_it->cluster.fCenterY.front();  // frontY of centers
        child_sTime = child_it->cluster.tMeasuredTime.front().toNSec();
        child_eTime = child_it->cluster.tMeasuredTime.back().toNSec();

        child_index = child_it->index;
        float stepWidth = getLength(this_centerX, this_centerY, child_centerX, child_centerY);

        // 計測時間が重なり、中心間距離が歩幅以下ならひもづける
        if ((getTime(child_sTime) - getTime(this_sTime)) >= 0 &&
            (getTime(this_eTime) + (double)OVERLAPTIME_ROOM - getTime(child_sTime)) > 0)
        {
          if (stepWidth < HUMAN_STEP_WIDTH)
          {
            // ただし、現在計測中のクラスタともひもづく可能性がある場合は
            // まだひもづけを行わない
            bool isConnectSensingCluster = false;
            std::list< Cluster >::iterator s_it;
            for (s_it = sensingClusterList.begin(); s_it != sensingClusterList.end(); s_it++)
            {
              // center, sensing time
              float s_centerX = s_it->fCenterX.front();  // frontX of centers
              float s_centerY = s_it->fCenterY.front();  // frontY of centers
              uint64_t s_sTime = s_it->tMeasuredTime.front().toNSec();
              uint64_t s_eTime = s_it->tMeasuredTime.back().toNSec();

              float s_stepWidth = getLength(this_centerX, this_centerY, s_centerX, s_centerY);

              // 計測時間、中心間距離をチェック
              if ((getTime(s_sTime) - getTime(this_sTime)) >= 0 &&
                  (getTime(this_eTime) + (double)OVERLAPTIME_ROOM - getTime(s_sTime)) > 0)
              {
                if (s_stepWidth < HUMAN_STEP_WIDTH)
                {
                  isConnectSensingCluster = true;
                  break;
                }
              }
            }

            // 計測中のクラスタとひもづく可能性が無い場合はひもづける
            if (!isConnectSensingCluster)
            {
              this_it->child_it.push_back(child_it);
              this_it->child_index.push_back(child_index);
            }
          }
        }
        else
        {
          break;
        }
      }
    }
  }

  //------------------------------------------------------------------------------
  // get start node which is not linked by other node
  std::vector< std::list< Node >::iterator > startNodes;
  std::vector< int > visitFlag;
  for (unsigned int i = 0; i < trajectoryTreeClusterIDMax + 1; i++)
  {
    visitFlag.push_back(0);
  }
  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    // check if this node has child nodes
    for (unsigned int i = 0; i < this_it->child_index.size(); i++)
    {
      int child_index = this_it->child_index[i];
      visitFlag[child_index] = 1;
    }
  }
  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    if (visitFlag[this_it->index] == 0)
    {
      startNodes.push_back(this_it);
    }
  }

  //------------------------------------------------------------------------------
  // get most longest root of each connections in TrajectoryTree
  // reset cost and isLeaf flag
  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    this_it->cost = -1;
    this_it->isLeaf = true;
  }
  for (unsigned int i = 0; i < startNodes.size(); i++)
  {
    float maxCost = getMaxCost(startNodes[i]);
  }

  // make humanTrajectory
  bool isLeaf;
  humanTrajectory.clear();
  for (unsigned int i = 0; i < startNodes.size(); i++)
  {
    this_it = startNodes[i];
    std::vector< Cluster > tempHumanTrajectory;

    while (1)
    {
      Cluster tempCluster;
      tempCluster = this_it->cluster;
      tempHumanTrajectory.push_back(tempCluster);

      isLeaf = this_it->isLeaf;
      if (isLeaf)
        break;
      else
        this_it = this_it->genuin_child_it;
    }
    if (tempHumanTrajectory.size() > 1)
    {
      humanTrajectory.push_back(tempHumanTrajectory);
    }
  }
}

float getMaxCost(std::list< Node >::iterator it)
{
  // if the cost already calculated, return the value
  if (it->cost != -1)
  {
    return it->cost;
  }
  else
  {
    if (it->child_it.size() == 0)
    {  // if node is leaf
      float cost = 0.0;
      it->cost = cost;
      it->isLeaf = true;
      return cost;
    }
    else
    {
      // 子ノードが複数存在し、かつ全ての子ノードが孫ノードを持たないときは
      // 子ノードまでのコストは計算せず、現在のノードまでとする
      bool isHaveGrandChild = false;
      if (it->child_it.size() > 1)
      {
        for (unsigned int i = 0; i < it->child_it.size(); i++)
        {
          int grandChildNum = it->child_it[i]->child_index.size();
          if (grandChildNum > 0)
          {
            isHaveGrandChild = true;
            break;
          }
        }
      }

      if (it->child_it.size() > 1 && !isHaveGrandChild)
      {
        // 孫ノードが存在しない場合、現在ノードまでで終了
        float cost = 0.0;
        it->cost = cost;
        it->isLeaf = true;
        return cost;
      }
      else
      {
        float mostLargeCost = FLT_MIN;
        std::list< Node >::iterator child_it;
        for (unsigned int i = 0; i < it->child_it.size(); i++)
        {
          float cost = 1.0 + getMaxCost(it->child_it[i]);
          if (cost >= mostLargeCost)
          {
            mostLargeCost = cost;
            child_it = it->child_it[i];
          }
        }
        // update
        it->cost = mostLargeCost;
        it->genuin_child_it = child_it;
        it->isLeaf = false;
        return mostLargeCost;
      }
    }
  }
}

void sendDetectedCluster()
{
  tms_msg_ss::fss_detected_cluster_data fss_detected_cluster_data;
  //------------------------------------------------------------------
  // Publish message
  fss_detected_cluster_data.header.frame_id = "fss_detected_cluster_data";
  fss_detected_cluster_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_detected_cluster_data.tMeasuredTime = GLOBAL_LAST_TIME;

  list< Cluster >::iterator it;
  for (it = detectedClusterList.begin(); it != detectedClusterList.end(); it++)
  {
    tms_msg_ss::fss_observed_datas fss_observed_datas;
    fss_observed_datas.tStartTime.push_back(it->tMeasuredTime.front());
    fss_observed_datas.tEndTime.push_back(it->tMeasuredTime.back());
    fss_observed_datas.fCenterX.push_back(it->fAveCenterX());
    fss_observed_datas.fCenterY.push_back(it->fAveCenterY());
    fss_detected_cluster_data.cluster.push_back(fss_observed_datas);
  }
  rosDetectedCluster.publish(fss_detected_cluster_data);
}

void sendPersonData()
{
  tms_msg_ss::fss_person_trajectory_data fss_person_trajectory_data;
  //------------------------------------------------------------------
  // Publish message
  fss_person_trajectory_data.header.frame_id = "fss_person_trajectory_data";
  fss_person_trajectory_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_person_trajectory_data.tMeasuredTime = GLOBAL_LAST_TIME;
  fss_person_trajectory_data.tMeasuredStartTime = GLOBAL_START_TIME;
  fss_person_trajectory_data.tMeasuredLastTime = GLOBAL_LAST_TIME;

  // trajectory
  for (unsigned int i = 0; i < humanTrajectory.size(); i++)
  {
    tms_msg_ss::fss_observed_datas fss_observed_datas;
    for (unsigned int j = 0; j < humanTrajectory[i].size(); j++)
    {
      fss_observed_datas.tStartTime.push_back(humanTrajectory[i][j].tMeasuredTime.front());
      fss_observed_datas.tEndTime.push_back(humanTrajectory[i][j].tMeasuredTime.back());
      fss_observed_datas.fCenterX.push_back(humanTrajectory[i][j].fAveCenterX());
      fss_observed_datas.fCenterY.push_back(humanTrajectory[i][j].fAveCenterY());
    }
    fss_person_trajectory_data.trajectory.push_back(fss_observed_datas);
  }

  // for TMS
  tms_msg_db::tmsdb_data msgTmsdbData;
  for (unsigned int i = 0; i < humanTrajectory.size(); i++)
  {
    for (unsigned int j = 0; j < humanTrajectory[i].size(); j++)
    {
      msgTmsdbData.tMeasuredTime = humanTrajectory[i][j].tMeasuredTime.front();
      msgTmsdbData.iType = TYPE_PERSON;
      msgTmsdbData.iID = ID_PERSON1;
      msgTmsdbData.fX = humanTrajectory[i][j].fAveCenterX();
      msgTmsdbData.fY = humanTrajectory[i][j].fAveCenterY();
      msgTmsdbData.fZ = 0;
      msgTmsdbData.fTheta = 0;
      msgTmsdbData.iPlace = ID_NONE;
      msgTmsdbData.iState = ID_NONE;

      fss_person_trajectory_data.msgTMSInfo.push_back(msgTmsdbData);
    }
  }
  rosPersonTrajectory.publish(fss_person_trajectory_data);
}

void displayCompareList()
{
  int i;
  std::list< Cluster >::iterator it;

  printf("-----------------\n");
  i = 0;
  for (it = sensingClusterList.begin(); it != sensingClusterList.end(); it++)
  {
    printf("■ sensingClusterList[%d]\n", i);
    printf("iID: %d\n", it->iID);
    printf("iType: %d\n", it->iType);
    printf("fLikelihood: %f\n", it->fLikelihood);
    printf("fSizeMax: %f\n", it->fSizeMax);
    printf("fSizeMin: %f\n", it->fSizeMin);
    printf("----\n");
    printf("iNumPoint: %d\n", it->iNumPoint.back());
    printf("fCenter: (%f, %f)\n", it->fCenterX.back(), it->fCenterY.back());
    printf("fMiddle: (%f, %f)\n", it->fMiddleX.back(), it->fMiddleY.back());
    printf("fRectangleMin: (%f, %f)\n", it->fRectangleMinX.back(), it->fRectangleMinY.back());
    printf("fRectangleMax: (%f, %f)\n", it->fRectangleMaxX.back(), it->fRectangleMaxY.back());
    printf("-----------------\n");
    i++;
  }
}

void displayDetectedClusterList()
{
  printf("------------------------\n");
  printf("detectedClusterList: %ld\n", detectedClusterList.size());

  int count = 0;
  std::list< Cluster >::iterator it;
  for (it = detectedClusterList.begin(); it != detectedClusterList.end(); it++)
  {
    float x = it->fCenterX.back();
    float y = it->fCenterY.back();
    double sTime = (double)getTime(it->tMeasuredTime.front().toNSec());
    double eTime = (double)getTime(it->tMeasuredTime.back().toNSec());
    printf("[%2d]: %f, %f, sensingTime: %lf, sTime: %lf, eTime: %lf\n", count, x, y, it->sensingTime, sTime, eTime);
    count++;
  }
  printf("\n");
}

void displayTrajectoryTree()
{
  printf("------------------------\n");
  printf("TrajectoryTree:\n");

  std::list< Node >::iterator this_it, child_it;

  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    printf("[%d]: ", this_it->index);
    for (unsigned int i = 0; i < this_it->child_it.size(); i++)
    {
      child_it = this_it->child_it[i];
      printf("[%d] ", child_it->index);
    }
    printf("\n");
  }
  printf("\n");
}

void displayHumanTrajectory()
{
  printf("------------------------\n");
  printf("humanTrajectory:\n");

  for (unsigned int i = 0; i < humanTrajectory.size(); i++)
  {
    for (unsigned int j = 0; j < humanTrajectory[i].size(); j++)
    {
      double sensingTime = humanTrajectory[i][j].sensingTime;
      float fSizeMax = humanTrajectory[i][j].fSizeMax;
      float fAveCenterX = humanTrajectory[i][j].fAveCenterX();
      float fAveCenterY = humanTrajectory[i][j].fAveCenterY();

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
          count += humanTrajectory[k].size();
        }
        index = count + j + 1;
      }
      printf("[%d][%d]:\t%lf\t%f\t(%f,%f)\n", i, j, sensingTime, fSizeMax, fAveCenterX, fAveCenterY);
    }
    printf("\n");
  }
}

void outputTrajectoryTreeData()
{
  //------------------------------------------------------------------------------
  // trajectoryTree
  ofs1 << "trajectoryTree" << std::endl;

  std::list< Node >::iterator this_it, child_it;
  for (this_it = trajectoryTree.begin(); this_it != trajectoryTree.end(); this_it++)
  {
    ofs1 << "[" << this_it->index << "]: ";
    for (unsigned int i = 0; i < this_it->child_it.size(); i++)
    {
      child_it = this_it->child_it[i];
      ofs1 << "[" << child_it->index << "] ";
    }
    ofs1 << std::endl;
  }
  ofs1 << std::endl;
}
