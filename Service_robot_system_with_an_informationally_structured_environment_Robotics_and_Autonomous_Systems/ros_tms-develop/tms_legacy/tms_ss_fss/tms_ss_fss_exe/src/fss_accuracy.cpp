//------------------------------------------------------------------------------
// @file   : fss_accuracy.cpp
// @brief  : measure lrf accuracy
// @author : Yoonseok Pyo
// @version: Ver0.2 (since 2012.07.02)
// @date   : 2012.07.13
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tms_ss_fss_exe/common.h>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;

int iLrf_scan_max_count = 721;

float fAverageDistance[5] = {
    0.0,
};
float fAverageIntensity[5] = {
    0.0,
};

float fVarianceDistance[5] = {
    0.0,
};
float fVarianceIntensity[5] = {
    0.0,
};

float fStandardDeviationDistance[5] = {
    0.0,
};
float fStandardDeviationIntensity[5] = {
    0.0,
};

int iCountMaxNum = 1000;
int iCountNum = 0;

bool bAverageMode = true;
bool bVarianceMode = false;
bool bStandardDeviationMode = false;

//------------------------------------------------------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void lrfCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //--------------------------------------------------------------------------
  // int iDataSet[5] = {106, 180, 360, 540, 615};
  // int iDataSet[5] = {180, 254, 360, 466, 540};
  // int iDataSet[5] = {225, 286, 360, 434, 495};
  int iDataSet[5] = {254, 304, 360, 416, 466};

  //--------------------------------------------------------------------------
  if (bAverageMode)
  {
    if (iCountNum < iCountMaxNum)
    {
      for (int i = 0; i < 5; i++)
      {
        fAverageDistance[i] += msg->ranges[iDataSet[i] - 1 - i] * 1000;  // M -> mm
        fAverageIntensity[i] += msg->intensities[iDataSet[i] - 1 - i];
      }
      iCountNum++;
    }

    if (iCountNum == iCountMaxNum)
    {
      for (int i = 0; i < 5; i++)
      {
        fAverageDistance[i] /= iCountMaxNum;
        fAverageIntensity[i] /= iCountMaxNum;
      }

      printf("\niCountNum = %d\n", iCountNum);
      for (int i = 0; i < 5; i++)
      {
        printf("fAverageDistance[%d] = %f\n", iDataSet[i], fAverageDistance[i]);
      }
      for (int i = 0; i < 5; i++)
      {
        printf("fAverageIntensity[%d] = %f\n", iDataSet[i], fAverageIntensity[i]);
      }
      bAverageMode = false;
      bVarianceMode = true;
      iCountNum = 0;
    }
  }

  //--------------------------------------------------------------------------
  if (bVarianceMode)
  {
    float fDeviation = 0.0;

    if (iCountNum < iCountMaxNum)
    {
      for (int i = 0; i < 5; i++)
      {
        fDeviation = ((msg->ranges[iDataSet[i] - 1 - i] * 1000) - fAverageDistance[i]);
        fVarianceDistance[i] += fDeviation * fDeviation;

        fDeviation = ((msg->intensities[iDataSet[i] - 1 - i]) - fAverageIntensity[i]);
        fVarianceIntensity[i] += fDeviation * fDeviation;
      }
      iCountNum++;
    }

    if (iCountNum == iCountMaxNum)
    {
      for (int i = 0; i < 5; i++)
      {
        fVarianceDistance[i] /= iCountMaxNum;
        fVarianceIntensity[i] /= iCountMaxNum;
      }

      for (int i = 0; i < 5; i++)
      {
        fStandardDeviationDistance[i] = sqrt(fVarianceDistance[i]);
        fStandardDeviationIntensity[i] = sqrt(fVarianceIntensity[i]);
      }

      printf("\niCountNum = %d\n", iCountNum);
      for (int i = 0; i < 5; i++)
      {
        printf("fVarianceDistance[%d] = %f\n", iDataSet[i], fVarianceDistance[i]);
      }
      for (int i = 0; i < 5; i++)
      {
        printf("fVarianceIntensity[%d] = %f\n", iDataSet[i], fVarianceIntensity[i]);
      }

      printf("\niCountNum = %d\n", iCountNum);
      for (int i = 0; i < 5; i++)
      {
        printf("fStandardDeviationDistance[%d] = %f\n", iDataSet[i], fStandardDeviationDistance[i]);
      }
      for (int i = 0; i < 5; i++)
      {
        printf("fStandardDeviationIntensity[%d] = %f\n", iDataSet[i], fStandardDeviationIntensity[i]);
      }
      ros::shutdown();
    }
  }

  //--------------------------------------------------------------------------
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_accuracy");
  ros::NodeHandle nh;
  rosSub = nh.subscribe("fss_raw_data", 10, lrfCallback);

  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}
