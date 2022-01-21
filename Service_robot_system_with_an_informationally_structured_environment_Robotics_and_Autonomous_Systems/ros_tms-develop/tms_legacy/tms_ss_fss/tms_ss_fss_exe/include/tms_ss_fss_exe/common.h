#ifndef COMMON_H
#define COMMON_H

//------------------------------------------------------------------------------
#include <vector>
#include <tms_msg_ss/fss_tf_datas.h>
//#include <tms_msg_ss/Cluster.h>
#include "Cluster.h"

//------------------------------------------------------------------------------
using namespace std;

//------------------------------------------------------------------------------
#define TYPE_ROBOT 1
#define TYPE_FUNITURE 2
#define TYPE_MOVABLE_FUNITURE 3
#define TYPE_PERSON 4
#define TYPE_OBJECT 5
#define TYPE_UNKNOWN_OBJECT 6
#define TYPE_MAP 7
#define TYPE_2D_MODEL 8
#define TYPE_3D_MODEL 9

#define ID_NONE 0

#define ID_SMARTPAL 1
#define ID_ROOMBA 2
#define ID_KKP 3

#define ID_FLOOR 11
#define ID_BED 12
#define ID_DESK 13
#define ID_TABLE 14
#define ID_CABINET1 15
#define ID_CABINET2 16

#define ID_WAGON1 21
#define ID_WAGON2 22
#define ID_CHAIR1 23
#define ID_CHAIR2 24
#define ID_WHEELCHAIR 25

#define ID_PERSON1 31
#define ID_PERSON2 32
#define ID_PERSON3 33
#define ID_PERSON4 34
#define ID_PERSON5 35

#define ID_NAMACHA 51
#define ID_CHIPSTAR1 52
#define ID_CHIPSTAR2 53
#define ID_CHIPSTAR3 54
#define ID_BOTTLE 55
#define ID_COINBANK 56
#define ID_WATER 57
#define ID_PETBOTTLE 58
#define ID_BOOK 59
#define ID_CALPIS 60

#define ID_UNKNOWN_OBJECT1 201
#define ID_UNKNOWN_OBJECT2 202
#define ID_UNKNOWN_OBJECT3 203
#define ID_UNKNOWN_OBJECT4 204
#define ID_UNKNOWN_OBJECT5 205

#define NAME_NAMACHA "namach"
#define NAME_CHIPSTAR1 "chipstar1"
#define NAME_CHIPSTAR2 "chipstar2"
#define NAME_CHIPSTAR3 "chipstar3"
#define NAME_BOTTLE "bottle"
#define NAME_COINBANK "coinbank"
#define NAME_WATER "water"
#define NAME_PETBOTTLE "petbottle"
#define NAME_BOOK "book"
#define NAME_CALPIS "calpis"

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)

//------------------------------------------------------------------------------
#define STATE_NOT_EXIST 0
#define STATE_EXIST 1
#define STATE_STOP 1
#define STATE_MOVE 2

#define SMARTPAL_R 420  // mm
#define ROOMBA_R 200    // mm
#define CHAIR_R 350     // mm
#define WAGON_R 370     // mm

#define MAXSTANDARDDEVIATION 35.0  // mm

//------------------------------------------------------------------------------
typedef struct
{
  float fX, fY;
} POINT;

//------------------------------------------------------------------------------
typedef struct
{
  float fX0, fY0;
  float fX1, fY1;
} LINE;

//------------------------------------------------------------------------------
typedef struct
{
  float fX1, fY1;
  float fX2, fY2;
} COORDINATE;

//------------------------------------------------------------------------------
typedef struct
{
  int iGroupID;
  bool bIsReflect;
  bool bIsForwardPoint;
  float fDistance;
  float fIntensity;
  float fAcuteAngle;
  COORDINATE stCoordinate;
} LRF_GROUP;

//------------------------------------------------------------------------------
typedef struct
{
  bool bIsReflect;
  bool bIsForwardPoint;
  float fDistance;
  float fIntensity;
  float fIntrinsicIntensity;
  float fAcuteAngle;
  COORDINATE stCoordinate;
} LRF_DATA;

//------------------------------------------------------------------------------
typedef struct
{
  float fDistance;
  float fIntensity;
  float fAcuteAngle;
} LRF;

//------------------------------------------------------------------------------
typedef struct
{
  std::vector< LRF > vstRawDataQueue;
} QUEUE;

//------------------------------------------------------------------------------
typedef struct
{
  float fPointX;
  float fPointY;
  uint32_t iIntensityCode;
} BOUNDARY_POINT;

//------------------------------------------------------------------------------
typedef struct
{
  uint32_t iStartPoint;
  uint32_t iEndPoint;
  float fLength;
  uint32_t iIntensityCode;
} POINT_LIST;

//------------------------------------------------------------------------------
typedef struct
{
  vector< uint32_t > viScanID;
  vector< float > vfDistance;
  vector< float > vfX;
  vector< float > vfY;
} POINT_GROUP;

//------------------------------------------------------------------------------
typedef struct
{
  ros::Time tMeasuredTime;
  vector< uint32_t > viID;
  vector< float > vfClusterCenterX;
  vector< float > vfClusterCenterY;
  vector< float > vfClusterSize;
  vector< float > vfAvgIntrinsicIntensity;
  vector< tms_msg_ss::fss_tf_datas > vstDatas;
} CLASS_DATA;

//------------------------------------------------------------------------------
typedef struct
{
  int mt_index;
  int gt_index;
  float score;
} Matching;

//------------------------------------------------------------------------------
class Node
{
public:
  int index;
  float cost;
  bool isLeaf;
  Cluster cluster;
  std::list< Node >::iterator genuin_child_it;

  std::vector< int > child_index;
  std::vector< std::list< Node >::iterator > child_it;
};

//------------------------------------------------------------------------------
#endif  // COMMON_H
