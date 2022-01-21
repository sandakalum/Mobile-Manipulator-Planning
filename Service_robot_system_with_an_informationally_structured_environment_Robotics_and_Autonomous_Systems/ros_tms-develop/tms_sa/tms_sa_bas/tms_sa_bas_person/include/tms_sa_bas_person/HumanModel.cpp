//------------------------------------------------------------------------------
// @file   : HumanModel.cpp
// @brief  : keep human's behavior, position and another parameters
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.11.13)
// @date   : 2012.11.13
//------------------------------------------------------------------------------

#include "HumanModel.h"

#define SIGMA 2.1  // σ

void HumanModel::calcBaseStepCycle(tms_msg_ss::fss_person_trajectory_data humanTrajectory)
{
  //------------------------------------------------------------------------------
  // copy sample stepCycle to stepCycleData
  for (unsigned int i = 0; i < humanTrajectory.trajectory.size(); i++)
  {
    std::vector< SampleData > tempStepCycleData;
    for (unsigned int j = 0; j < humanTrajectory.trajectory[i].fCenterX.size(); j++)
    {
      unsigned int k = j + 1;
      if (k < humanTrajectory.trajectory[i].fCenterX.size())
      {
        // calculate stepCycle
        double tempStepCycle;
        uint64_t this_sTime, next_sTime;
        SampleData tempData;

        this_sTime = humanTrajectory.trajectory[i].tStartTime[j].toNSec();
        next_sTime = humanTrajectory.trajectory[i].tStartTime[k].toNSec();
        tempStepCycle = (double)(next_sTime - this_sTime) / 1.0e9;

        tempData.value = tempStepCycle;
        tempData.isValid = true;
        tempStepCycleData.push_back(tempData);
      }
      else
        break;
    }
    stepCycleData.push_back(tempStepCycleData);
  }

  // printf("------------------------------------------------\n");
  // printf("calcStepCycle:\n");
  // printf("------------------------------------------------\n");
  //------------------------------------------------------------------------------
  // calculate average and var in nσ
  double n = SIGMA;

  for (unsigned int i = 0; i < stepCycleData.size(); i++)
  {
    // printf("humanTrajectory[%d]: data_num: %d\n", i, stepCycleData[i].size());
    int count;
    double sigma;
    double stepCycle, stepCycleVar, sumStepCycle, sumStepCycleVar;

    while (1)
    {
      count = 0;
      stepCycle = stepCycleVar = sumStepCycle = sumStepCycleVar = 0.0;

      if (stepCycleData[i].size() > 0)
      {
        for (unsigned int j = 0; j < stepCycleData[i].size(); j++)
        {
          if (stepCycleData[i][j].isValid)
          {
            sumStepCycle += stepCycleData[i][j].value;
            sumStepCycleVar += stepCycleData[i][j].value * stepCycleData[i][j].value;
            ;
            count++;
          }
        }

        stepCycle = sumStepCycle / (double)count;
        stepCycleVar = (sumStepCycleVar / (double)count) - stepCycle * stepCycle;
        sigma = sqrt(stepCycleVar);

        // printf("stepCycle: %lf, stepCycleVar: %lf, sigma: %lf\n", stepCycle, stepCycleVar, sigma);

        // eliminate sample data which is out of nσ
        bool isFinished = true;
        for (unsigned int j = 0; j < stepCycleData[i].size(); j++)
        {
          if (stepCycleData[i][j].isValid)
          {
            if (stepCycleData[i][j].value <= stepCycle - n * sigma ||
                stepCycle + n * sigma <= stepCycleData[i][j].value)
            {
              stepCycleData[i][j].isValid = false;
              isFinished = false;
            }
          }
        }
        if (isFinished)
        {
          baseStepCycle.push_back(stepCycle);
          baseStepCycleVar.push_back(stepCycleVar);
          break;
        }
      }
      else
      {
        // printf("【Error】 No data.\n");
        // printf("stepCycle: %lf, stepCycleVar: %lf, sigma: %lf\n", stepCycle, stepCycleVar, sigma);
        baseStepCycle.push_back(stepCycle);
        baseStepCycleVar.push_back(stepCycleVar);
        break;
      }
    }

    //------------------------------------------------------------------------------
    // display stepCycle
    //		if(stepCycleData[i].size() > 0){
    //			for(unsigned int j=0 ; j<stepCycleData[i].size() ; j++){
    //				printf("[%d]-[%d]: %lf", j+1, j+2, stepCycleData[i][j].value);
    //				if(!stepCycleData[i][j].isValid)
    //					printf(" -- false");
    //				printf("\n");
    //			}
    //		}
    //
    //		printf("--\n");
    //		printf("stepCycle: %lf\n", stepCycle);
    //		printf("stepCycleVar: %lf\n", stepCycleVar);
    //		printf("------------------------------------------------\n");
  }
}

void HumanModel::calcBaseGaitCycle(tms_msg_ss::fss_person_trajectory_data humanTrajectory)
{
  //------------------------------------------------------------------------------
  // copy sample gaitCycle to gaitCycleData
  for (unsigned int i = 0; i < humanTrajectory.trajectory.size(); i++)
  {
    std::vector< SampleData > tempGaitCycleData;
    for (unsigned int j = 0; j < humanTrajectory.trajectory[i].fCenterX.size(); j++)
    {
      unsigned int k = j + 2;
      if (k < humanTrajectory.trajectory[i].fCenterX.size())
      {
        // calculate gaitCycle
        double tempGaitCycle;
        uint64_t this_sTime, next_sTime;
        SampleData tempData;

        this_sTime = humanTrajectory.trajectory[i].tStartTime[j].toNSec();
        next_sTime = humanTrajectory.trajectory[i].tStartTime[k].toNSec();
        tempGaitCycle = (double)(next_sTime - this_sTime) / 1.0e9;

        tempData.value = tempGaitCycle;
        tempData.isValid = true;
        tempGaitCycleData.push_back(tempData);
      }
      else
        break;
    }
    gaitCycleData.push_back(tempGaitCycleData);
  }

  // printf("------------------------------------------------\n");
  // printf("calcGaitCycle:\n");
  // printf("------------------------------------------------\n");
  //------------------------------------------------------------------------------
  // calculate average and var in nσ
  double n = SIGMA;

  for (unsigned int i = 0; i < gaitCycleData.size(); i++)
  {
    // printf("humanTrajectory[%d]: data_num: %d\n", i, gaitCycleData[i].size());
    int count;
    double sigma;
    double gaitCycle, gaitCycleVar, sumGaitCycle, sumGaitCycleVar;

    while (1)
    {
      count = 0;
      gaitCycle = gaitCycleVar = sumGaitCycle = sumGaitCycleVar = 0.0;

      if (gaitCycleData[i].size() > 0)
      {
        for (unsigned int j = 0; j < gaitCycleData[i].size(); j++)
        {
          if (gaitCycleData[i][j].isValid)
          {
            sumGaitCycle += gaitCycleData[i][j].value;
            sumGaitCycleVar += gaitCycleData[i][j].value * gaitCycleData[i][j].value;
            ;
            count++;
          }
        }

        gaitCycle = sumGaitCycle / (double)count;
        gaitCycleVar = (sumGaitCycleVar / (double)count) - gaitCycle * gaitCycle;
        sigma = sqrt(gaitCycleVar);

        // printf("gaitCycle: %lf, gaitCycleVar: %lf, sigma: %lf\n", gaitCycle, gaitCycleVar, sigma);

        // eliminate sample data which is out of nσ
        bool isFinished = true;
        for (unsigned int j = 0; j < gaitCycleData[i].size(); j++)
        {
          if (gaitCycleData[i][j].isValid)
          {
            if (gaitCycleData[i][j].value <= gaitCycle - n * sigma ||
                gaitCycle + n * sigma <= gaitCycleData[i][j].value)
            {
              gaitCycleData[i][j].isValid = false;
              isFinished = false;
            }
          }
        }
        if (isFinished)
        {
          baseGaitCycle.push_back(gaitCycle);
          baseGaitCycleVar.push_back(gaitCycleVar);
          break;
        }
      }
      else
      {
        // printf("【Error】 No data.\n");
        // printf("gaitCycle: %lf, gaitCycleVar: %lf, sigma: %lf\n", gaitCycle, gaitCycleVar, sigma);
        baseGaitCycle.push_back(gaitCycle);
        baseGaitCycleVar.push_back(gaitCycleVar);
        break;
      }
    }

    //------------------------------------------------------------------------------
    // display gaitCycle
    //		if(gaitCycleData[i].size() > 0){
    //			for(unsigned int j=0 ; j<gaitCycleData[i].size() ; j++){
    //				printf("[%d]-[%d]: %lf", j+1, j+3, gaitCycleData[i][j].value);
    //				if(!gaitCycleData[i][j].isValid)
    //					printf(" -- false");
    //				printf("\n");
    //			}
    //		}
    //
    //		printf("--\n");
    //		printf("gaitCycle: %lf\n", gaitCycle);
    //		printf("gaitCycleVar: %lf\n", gaitCycleVar);
    //		printf("------------------------------------------------\n");
  }
}

void HumanModel::calcBaseStepCycleByAll()
{
  //------------------------------------------------------------------------------
  // copy each stepCycleData to stepCycleDataByAll
  for (unsigned int i = 0; i < stepCycleData.size(); i++)
  {
    for (unsigned int j = 0; j < stepCycleData[i].size(); j++)
    {
      SampleData tempStepCycleDataByAll;
      tempStepCycleDataByAll.value = stepCycleData[i][j].value;
      tempStepCycleDataByAll.isValid = true;
      stepCycleDataByAll.push_back(tempStepCycleDataByAll);
    }
  }

  //	printf("------------------------------------------------\n");
  //	printf("calcStepCycleByAll:\n");
  //	printf("------------------------------------------------\n");
  //	printf("humanTrajectory all data: data_num: %d\n",stepCycleDataByAll.size());
  //------------------------------------------------------------------------------
  // calculate average and var in nσ
  double n = SIGMA;

  int count;
  double sigma;
  double stepCycle, stepCycleVar, sumStepCycle, sumStepCycleVar;

  while (1)
  {
    count = 0;
    stepCycle = stepCycleVar = sumStepCycle = sumStepCycleVar = 0.0;

    if (stepCycleDataByAll.size() > 0)
    {
      for (unsigned int i = 0; i < stepCycleDataByAll.size(); i++)
      {
        if (stepCycleDataByAll[i].isValid)
        {
          sumStepCycle += stepCycleDataByAll[i].value;
          sumStepCycleVar += stepCycleDataByAll[i].value * stepCycleDataByAll[i].value;
          ;
          count++;
        }
      }

      stepCycle = sumStepCycle / (double)count;
      stepCycleVar = (sumStepCycleVar / (double)count) - stepCycle * stepCycle;
      sigma = sqrt(stepCycleVar);

      // printf("stepCycle: %lf, stepCycleVar: %lf, sigma: %lf\n", stepCycle, stepCycleVar, sigma);

      // eliminate sample data which is out of nσ
      bool isFinished = true;
      for (unsigned int i = 0; i < stepCycleDataByAll.size(); i++)
      {
        if (stepCycleDataByAll[i].isValid)
        {
          if (stepCycleDataByAll[i].value <= stepCycle - n * sigma ||
              stepCycle + n * sigma <= stepCycleDataByAll[i].value)
          {
            stepCycleDataByAll[i].isValid = false;
            isFinished = false;
          }
        }
      }
      if (isFinished)
      {
        baseStepCycleByAll = stepCycle;
        baseStepCycleVarByAll = stepCycleVar;
        break;
      }
    }
    else
    {
      // printf("【Error】 No data.\n");
      // printf("stepCycle: %lf, stepCycleVar: %lf, sigma: %lf\n", stepCycle, stepCycleVar, sigma);
      baseStepCycleByAll = stepCycle;
      baseStepCycleVarByAll = stepCycleVar;
      break;
    }
  }

  //------------------------------------------------------------------------------
  // display stepCycle
  //	if(stepCycleDataByAll.size() > 0){
  //		for(unsigned int i=0 ; i<stepCycleDataByAll.size() ; i++){
  //			printf("[%d]-[%d]: %lf", i+1, i+2, stepCycleDataByAll[i].value);
  //			if(!stepCycleDataByAll[i].isValid)
  //				printf(" -- false");
  //			printf("\n");
  //		}
  //	}
  //
  //	printf("--\n");
  //	printf("stepCycle: %lf\n", stepCycle);
  //	printf("stepCycleVar: %lf\n", stepCycleVar);
  //	printf("------------------------------------------------\n");
}

void HumanModel::calcBaseGaitCycleByAll()
{
  //------------------------------------------------------------------------------
  // copy each gaitCycleData to gaitCycleDataByAll
  for (unsigned int i = 0; i < gaitCycleData.size(); i++)
  {
    for (unsigned int j = 0; j < gaitCycleData[i].size(); j++)
    {
      SampleData tempGaitCycleDataByAll;
      tempGaitCycleDataByAll.value = gaitCycleData[i][j].value;
      tempGaitCycleDataByAll.isValid = true;
      gaitCycleDataByAll.push_back(tempGaitCycleDataByAll);
    }
  }

  //	printf("------------------------------------------------\n");
  //	printf("calcGaitCycleByAll:\n");
  //	printf("------------------------------------------------\n");
  //	printf("humanTrajectory all data: data_num: %d\n",gaitCycleDataByAll.size());
  //------------------------------------------------------------------------------
  // calculate average and var in nσ
  double n = SIGMA;

  int count;
  double sigma;
  double gaitCycle, gaitCycleVar, sumGaitCycle, sumGaitCycleVar;

  while (1)
  {
    count = 0;
    gaitCycle = gaitCycleVar = sumGaitCycle = sumGaitCycleVar = 0.0;

    if (gaitCycleDataByAll.size() > 0)
    {
      for (unsigned int i = 0; i < gaitCycleDataByAll.size(); i++)
      {
        if (gaitCycleDataByAll[i].isValid)
        {
          sumGaitCycle += gaitCycleDataByAll[i].value;
          sumGaitCycleVar += gaitCycleDataByAll[i].value * gaitCycleDataByAll[i].value;
          ;
          count++;
        }
      }

      gaitCycle = sumGaitCycle / (double)count;
      gaitCycleVar = (sumGaitCycleVar / (double)count) - gaitCycle * gaitCycle;
      sigma = sqrt(gaitCycleVar);

      // printf("gaitCycle: %lf, gaitCycleVar: %lf, sigma: %lf\n", gaitCycle, gaitCycleVar, sigma);

      // eliminate sample data which is out of nσ
      bool isFinished = true;
      for (unsigned int i = 0; i < gaitCycleDataByAll.size(); i++)
      {
        if (gaitCycleDataByAll[i].isValid)
        {
          if (gaitCycleDataByAll[i].value <= gaitCycle - n * sigma ||
              gaitCycle + n * sigma <= gaitCycleDataByAll[i].value)
          {
            gaitCycleDataByAll[i].isValid = false;
            isFinished = false;
          }
        }
      }
      if (isFinished)
      {
        baseGaitCycleByAll = gaitCycle;
        baseGaitCycleVarByAll = gaitCycleVar;
        break;
      }
    }
    else
    {
      // printf("【Error】 No data.\n");
      // printf("gaitCycle: %lf, gaitCycleVar: %lf, sigma: %lf\n", gaitCycle, gaitCycleVar, sigma);
      baseGaitCycleByAll = gaitCycle;
      baseGaitCycleVarByAll = gaitCycleVar;
      break;
    }
  }

  //------------------------------------------------------------------------------
  // display gaitCycle
  //	if(gaitCycleDataByAll.size() > 0){
  //		for(unsigned int i=0 ; i<gaitCycleDataByAll.size() ; i++){
  //			printf("[%d]-[%d]: %lf", i+1, i+2, gaitCycleDataByAll[i].value);
  //			if(!gaitCycleDataByAll[i].isValid)
  //				printf(" -- false");
  //			printf("\n");
  //		}
  //	}
  //
  //	printf("--\n");
  //	printf("gaitCycle: %lf\n", gaitCycle);
  //	printf("gaitCycleVar: %lf\n", gaitCycleVar);
  //	printf("------------------------------------------------\n");
}

void HumanModel::clearAllParameter()
{
  behaviorWalking = 0;
  behaviorSitting = 0;
  behaviorSleeping = 0;
  stepCycle = 0;
  fPosFoot1X = 0;
  fPosFoot1Y = 0;
  fPosFoot2X = 0;
  fPosFoot2Y = 0;
  fAccFoot1X = 0;
  fAccFoot1Y = 0;
  fAccFoot2X = 0;
  fAccFoot2Y = 0;
  baseStepCycleByAll = 0;
  baseStepCycleVarByAll = 0;
  baseGaitCycleByAll = 0;
  baseGaitCycleVarByAll = 0;

  baseStepCycle.clear();
  baseStepCycleVar.clear();
  baseGaitCycle.clear();
  baseGaitCycleVar.clear();
  stepCycleData.clear();
  gaitCycleData.clear();
  stepCycleDataByAll.clear();
  gaitCycleDataByAll.clear();
}
