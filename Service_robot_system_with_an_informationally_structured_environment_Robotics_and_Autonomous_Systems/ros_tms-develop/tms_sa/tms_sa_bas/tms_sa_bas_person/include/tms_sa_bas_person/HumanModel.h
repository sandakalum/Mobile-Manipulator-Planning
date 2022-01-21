//------------------------------------------------------------------------------
// @file   : HumanModel.h
// @brief  : keep human's behavior, position and another parameters
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.11.13)
// @date   : 2012.11.13
//------------------------------------------------------------------------------

#pragma once
#include <vector>
#include <tms_msg_ss/fss_person_trajectory_data.h>
#include "common.h"

class HumanModel
{
public:
  // variables
  int behaviorWalking;
  int behaviorSitting;
  int behaviorSleeping;

  float stepCycle;
  float fPosFoot1X;
  float fPosFoot1Y;
  float fPosFoot2X;
  float fPosFoot2Y;

  float fAccFoot1X;
  float fAccFoot1Y;
  float fAccFoot2X;
  float fAccFoot2Y;

  //--------------------------------------------------------------------------
  // for each humanTrajectory
  std::vector< double > baseStepCycle;
  std::vector< double > baseStepCycleVar;
  std::vector< double > baseGaitCycle;
  std::vector< double > baseGaitCycleVar;
  std::vector< vector< SampleData > > stepCycleData;
  std::vector< vector< SampleData > > gaitCycleData;

  //--------------------------------------------------------------------------
  // for humanTrajectory all data
  double baseStepCycleByAll;
  double baseStepCycleVarByAll;
  double baseGaitCycleByAll;
  double baseGaitCycleVarByAll;
  std::vector< SampleData > stepCycleDataByAll;
  std::vector< SampleData > gaitCycleDataByAll;

  //--------------------------------------------------------------------------
  // functions
  void calcBaseStepCycle(tms_msg_ss::fss_person_trajectory_data humanTrajectory);
  void calcBaseGaitCycle(tms_msg_ss::fss_person_trajectory_data humanTrajectory);
  void calcBaseStepCycleByAll();
  void calcBaseGaitCycleByAll();

  void clearAllParameter();
};
