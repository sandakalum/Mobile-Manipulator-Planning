#ifndef COMMON_H
#define COMMON_H

//------------------------------------------------------------------------------
#include <vector>

//------------------------------------------------------------------------------
using namespace std;

// HumanBehavior:
#define BEHAVIOR_LOST 0       //  0: lost
#define BEHAVIOR_WALKING 1    //  1: walking
#define BEHAVIOR_STAY_F 2     //  2: stay in the floor
#define BEHAVIOR_STAY_C 3     //  3: stay near the chair
#define BEHAVIOR_SITTING_C 4  //  4: sitting down on a chair
#define BEHAVIOR_SITTING_B 5  //  5: sitting down on a bed
#define BEHAVIOR_STAY_B 6     //  6: stay on a bed(sleeping)

typedef struct
{
  double value;
  bool isValid;
} SampleData;

typedef struct
{
  uint64_t sTime;
  uint64_t eTime;
  double sensingTime;
  float fCenterX;
  float fCenterY;
} SumSensingTime;

typedef struct
{
  double sTime;
  double eTime;
  int behavior;
  std::vector< float > fCenterX;
  std::vector< float > fCenterY;
} HumanBehavior;

//------------------------------------------------------------------------------
#endif  // COMMON_H
