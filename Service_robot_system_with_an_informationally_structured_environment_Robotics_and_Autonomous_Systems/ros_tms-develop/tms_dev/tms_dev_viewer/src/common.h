#ifndef COMMON_H
#define COMMON_H

//------------------------------------------------------------------------------
#include <vector>
using namespace std;

//------------------------------------------------------------------------------
#define ON true
#define OFF false

//------------------------------------------------------------------------------
#define MODE_GRID_SQUARE 1
#define MODE_GRID_CIRCLE 2
#define MODE_GRID_OFF 0

#define MODE_LASER_POINT 1
#define MODE_LASER_LINE 2
#define MODE_LASER_OFF 0

#define MODE_OCCLUSION_POINT 1
#define MODE_OCCLUSION_FACE 2
#define MODE_OCCLUSION_ON 3
#define MODE_OCCLUSION_OFF 0

#define MODE_CLUSTER_ON 1
#define MODE_CLUSTER_OFF 0

#define MODE_CLASS_ON 1
#define MODE_CLASS_OFF 0

#define MODE_UNKNOWN_CLASS_ON 1
#define MODE_UNKNOWN_CLASS_OFF 0

#define MODE_FURNITURE_ON 1
#define MODE_FURNITURE_OFF 0

#define MODE_PATH_ON 1
#define MODE_PATH_OFF 0
#define MODE_PLAN_PATH_ON 1
#define MODE_PLAN_PATH_OFF 0
#define MODE_ROBOT_PATH_ON 1
#define MODE_ROBOT_PATH_OFF 0
#define MODE_WAGON_PATH_ON 1
#define MODE_WAGON_PATH_OFF 0

#define MODE_RPS_MAP_ON 1
#define MODE_RPS_MAP_OFF 0

#define MODE_SMARTPAL_ON 1
#define MODE_SMARTPAL_OFF 0

#define MODE_ROOMBA_ON 1
#define MODE_ROOMBA_OFF 0

#define MODE_WAGON_ON 1
#define MODE_WAGON_OFF 0

#define MODE_CHAIR_ON 1
#define MODE_CHAIR_OFF 0

#define MODE_PERSON_TRAJECTORY_ON 1
#define MODE_PERSON_TRAJECTORY_OFF 0

#define MODE_UNKNOEN_OBJECT_ON 1
#define MODE_UNKNOEN_OBJECT_OFF 0

#define MODE_WAGON_ISS_ON 1
#define MODE_WAGON_ISS_OFF 0

#define MODE_WHEELCHAIR_ON 1
#define MODE_WHEELCHAIR_OFF 0

#define MODE_OBJECT_ON 1
#define MODE_OBJECT_OFF 0

#define MODE_INTENSITY_ON 1
#define MODE_INTENSITY_OFF 0

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

#define NAME_NAMACHA "namacha"
#define NAME_CHIPSTAR1 "chipstar1"
#define NAME_CHIPSTAR2 "chipstar2"
#define NAME_CHIPSTAR3 "chipstar3"
#define NAME_BOTTLE "bottle"
#define NAME_COINBANK "coinbank"
#define NAME_WATER "water"
#define NAME_PETBOTTLE "petbottle"
#define NAME_BOOK "book"
#define NAME_CALPIS "calpis"

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
  double dX, dY;
} POINT;

typedef struct
{
  double dX0, dY0;
  double dX1, dY1;
} LINE;

typedef struct
{
  float fDistance;
  float fIntensity;
  float fIntrinsicIntensity;
  float fAcuteAngle;
} LRF;

//------------------------------------------------------------------------------
#endif  // COMMON_H

//------------------------------------------------------------------------------
