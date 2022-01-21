//------------------------------------------------------------------------------
// @file   : fss_transform.cpp
// @brief  : Make tf data
// @author : Yoonseok Pyo
// @version: Ver0.6 (since 2012.06.01)
// @date   : 2012.10.04
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tms_ss_fss_exe/common.h>

#include <tms_msg_ss/fss_pre_data.h>
#include <tms_msg_ss/fss_tf_data.h>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;
ros::Publisher rosPub;

ros::Subscriber rosSub1;
ros::Publisher rosPub1;

ros::Subscriber rosSub2;
ros::Publisher rosPub2;

int iLrf_scan_max_count = 721;
int iBackgroundCount = 0;
float fBackgroundData[721] = {0};
float fBackgroundThreshold = 100.0;  // 100mm

//------------------------------------------------------------------------------
bool is_intersect(vector< LINE > f_vstA, vector< LINE > f_vstB)
{
  for (unsigned int i = 0; i < f_vstA.size(); ++i)
  {
    for (unsigned int j = 0; j < f_vstB.size(); ++j)
    {
      float cx = f_vstA[i].fX0;
      float cy = f_vstA[i].fY0;
      float fx = f_vstA[i].fX1 - f_vstA[i].fX0;
      float fy = f_vstA[i].fY1 - f_vstA[i].fY0;

      float ax = f_vstB[j].fX0;
      float ay = f_vstB[j].fY0;
      float ex = f_vstB[j].fX1 - f_vstB[j].fX0;
      float ey = f_vstB[j].fY1 - f_vstB[j].fY0;

      float t1 = ((cx * fy - cy * fx) - (ax * fy - ay * fx)) / (ex * fy - ey * fx);
      float t2 = ((ax * ey - ay * ex) - (cx * ey - cy * ex)) / (fx * ey - fy * ex);

      if (0. <= t1 && t1 <= 1. && 0. <= t2 && t2 <= 1.)
      {
        return true;
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
bool get_point_of_intersection(LINE f_stA, LINE f_stB, POINT* f_stP)
{
  float t;
  float s;
  float under = (f_stB.fY1 - f_stB.fY0) * (f_stA.fX1 - f_stA.fX0) - (f_stB.fX1 - f_stB.fX0) * (f_stA.fY1 - f_stA.fY0);

  if (under == 0)
    return false;

  float _t = (f_stB.fX1 - f_stB.fX0) * (f_stA.fY0 - f_stB.fY0) - (f_stB.fY1 - f_stB.fY0) * (f_stA.fX0 - f_stB.fX0);
  float _s = (f_stA.fX1 - f_stA.fX0) * (f_stA.fY0 - f_stB.fY0) - (f_stA.fY1 - f_stA.fY0) * (f_stA.fX0 - f_stB.fX0);

  t = _t / under;
  s = _s / under;

  if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0)
    return false;
  if (_t == 0 && _s == 0)
    return false;

  f_stP->fX = f_stA.fX0 + t * (f_stA.fX1 - f_stA.fX0);
  f_stP->fY = f_stA.fY0 + t * (f_stA.fY1 - f_stA.fY0);

  return true;
}

//------------------------------------------------------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void lrfCallback(const tms_msg_ss::fss_pre_data::ConstPtr& msg)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed sending of fss_tf_data #s");
  double StartTime = ros::Time::now().toSec();

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_tf_data fss_tf_data;

  LRF_DATA stLrfData;
  vector< LRF_DATA > vstLrfDataTemp;

  POINT stPoint;

  float fAngle = 0;
  float fLrf_set_x = 2000;  ////3000
  float fLrf_set_y = 0;

  LINE stMirror_line;
  LINE stLrf_Laser_line;
  POINT stIntersection_point;

  vector< uint8_t > vstReflect;
  vector< uint8_t > vstIsForwardPoint;
  vector< float > vstDistance;
  vector< float > vstIntensity;
  vector< float > vstIntrinsicIntensity;
  vector< float > vstAcuteAngle;
  vector< float > vstX1;
  vector< float > vstY1;
  vector< float > vstX2;
  vector< float > vstY2;

  unsigned int iLength = 0;

  //--------------------------------------------------------------------------
  // Make BackgroundData
  if (iBackgroundCount < 50)
  {
    iLength = msg->fDistance.size();
    for (unsigned int i = 0; i < iLength; i++)
    {
      fBackgroundData[i] = max(fBackgroundData[i], msg->fDistance[i]);
    }

    iBackgroundCount++;
    return;
  }

  //--------------------------------------------------------------------------
  // Make  bIsForwardPoint, bIsReflect, X1, Y1, X2, Y2
  iLength = msg->fDistance.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    if (msg->fDistance[i] + fBackgroundThreshold < fBackgroundData[i])
    {
      stLrfData.bIsForwardPoint = true;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }
    else
    {
      stLrfData.bIsForwardPoint = false;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }

    //----------------------------------------------------------------------
    fAngle = i * 0.25 * DEG2RAD;

    stMirror_line.fX0 = 0;
    stMirror_line.fY0 = 0;
    stMirror_line.fX1 = 0;
    stMirror_line.fY1 = 4000;

    stPoint.fX = stLrfData.fDistance * cos(fAngle) + fLrf_set_x;
    stPoint.fY = stLrfData.fDistance * sin(fAngle) + fLrf_set_y;

    stLrf_Laser_line.fX0 = fLrf_set_x;
    stLrf_Laser_line.fY0 = fLrf_set_y;
    stLrf_Laser_line.fX1 = stPoint.fX;
    stLrf_Laser_line.fY1 = stPoint.fY;

    //----------------------------------------------------------------------
    stLrfData.bIsReflect = get_point_of_intersection(stMirror_line, stLrf_Laser_line, &stIntersection_point);

    //----------------------------------------------------------------------
    if (stLrfData.bIsReflect)  // reflect laser
    {
      stPoint.fX = (stPoint.fX - stMirror_line.fX0) * -1 + stMirror_line.fX0;

      stLrfData.stCoordinate.fX1 = stIntersection_point.fX;
      stLrfData.stCoordinate.fY1 = stIntersection_point.fY;
      stLrfData.stCoordinate.fX2 = stPoint.fX;
      stLrfData.stCoordinate.fY2 = stPoint.fY;
    }
    else  // direct laser
    {
      stLrfData.stCoordinate.fX1 = stPoint.fX;
      stLrfData.stCoordinate.fY1 = stPoint.fY;
      stLrfData.stCoordinate.fX2 = stPoint.fX;
      stLrfData.stCoordinate.fY2 = stPoint.fY;
    }

    vstLrfDataTemp.push_back(stLrfData);

    //----------------------------------------------------------------------
    vstReflect.push_back(stLrfData.bIsReflect);
    vstIsForwardPoint.push_back(stLrfData.bIsForwardPoint);
    vstDistance.push_back(stLrfData.fDistance);
    vstIntensity.push_back(stLrfData.fIntensity);
    vstX1.push_back(stLrfData.stCoordinate.fX1);
    vstY1.push_back(stLrfData.stCoordinate.fY1);
    vstX2.push_back(stLrfData.stCoordinate.fX2);
    vstY2.push_back(stLrfData.stCoordinate.fY2);
  }

  //--------------------------------------------------------------------------
  // Make AcuteAngle data, AbsoluteIntensity data

  for (unsigned int i = 0; i < vstLrfDataTemp.size(); i++)
  {
    if (i <= 1 || i >= (vstLrfDataTemp.size() - 2))  //        if(i<=0 || i>=(vstLrfDataTemp.size()-1))
    {
      stLrfData.fAcuteAngle = 0.0;
      stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity;
    }
    else
    {
      float centerX = (vstLrfDataTemp[i - 1].stCoordinate.fX2 + vstLrfDataTemp[i + 1].stCoordinate.fX2) / 2;
      float centerY = (vstLrfDataTemp[i - 1].stCoordinate.fY2 + vstLrfDataTemp[i + 1].stCoordinate.fY2) / 2;
      float lengthC = get_length(centerX, centerY, fLrf_set_x, fLrf_set_y);

      float lengthB = get_length(vstLrfDataTemp[i - 1].stCoordinate.fX2, vstLrfDataTemp[i - 1].stCoordinate.fY2,
                                 vstLrfDataTemp[i + 1].stCoordinate.fX2, vstLrfDataTemp[i + 1].stCoordinate.fY2) /
                      2;

      float lengthA = vstLrfDataTemp[i + 1].fDistance;

      float theta = acos((lengthB * lengthB + lengthC * lengthC - lengthA * lengthA) / (2 * lengthB * lengthC));

      vstLrfDataTemp[i].fAcuteAngle = abs(abs(theta) - 1.5708);

      //
      if (vstLrfDataTemp[i].bIsReflect == true)
      {
        vstLrfDataTemp[i].fAcuteAngle = 0.7854;
      }
      //

      if (vstLrfDataTemp[i].fAcuteAngle < 0.0 || vstLrfDataTemp[i].fAcuteAngle > 1.5708)
      {
        stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity;
        stLrfData.fAcuteAngle = 0;

        if (vstLrfDataTemp[i].bIsReflect == true)
        {
          stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity * 2;
        }
      }
      else
      {
        float ir = pow(vstLrfDataTemp[i].fDistance, 0.287f);         // 0.27
        float ia = pow(cos(vstLrfDataTemp[i].fAcuteAngle), 0.196f);  // 0.22

        if (ia == 0)
          ia = 0.001;

        if (vstLrfDataTemp[i].fAcuteAngle < 1.3963 && vstLrfDataTemp[i].fIntensity < 5000)
          stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity * ir / ia / 10;
        else
          stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity * ir / 10;

        if (vstLrfDataTemp[i].bIsReflect == true)
        {
          stLrfData.fIntrinsicIntensity = vstLrfDataTemp[i].fIntensity * ir / ia / 10 * 1.2;
        }

        stLrfData.fAcuteAngle = vstLrfDataTemp[i].fAcuteAngle;
      }
    }
    vstAcuteAngle.push_back(stLrfData.fAcuteAngle);
    vstIntrinsicIntensity.push_back(stLrfData.fIntrinsicIntensity);
  }

  //--------------------------------------------------------------------------
  fss_tf_data.header.frame_id = "fss_tf_data";
  fss_tf_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_tf_data.tMeasuredTime = msg->tMeasuredTime;

  fss_tf_data.bIsReflect = vstReflect;
  fss_tf_data.bIsForwardPoint = vstIsForwardPoint;
  fss_tf_data.fDistance = vstDistance;
  fss_tf_data.fIntensity = vstIntensity;
  fss_tf_data.fIntrinsicIntensity = vstIntrinsicIntensity;
  fss_tf_data.fAcuteAngle = vstAcuteAngle;
  fss_tf_data.fX1 = vstX1;
  fss_tf_data.fY1 = vstY1;
  fss_tf_data.fX2 = vstX2;
  fss_tf_data.fY2 = vstY2;
  fss_tf_data.fY2 = vstY2;

  //--------------------------------------------------------------------------
  // Make Occlusion Area Data
  vector< LINE > vstOcclusion;
  vector< LINE > vstLaser;
  LINE stLaser;

  vector< POINT > vstOcculusion;
  vector< float > vstOcculusionX;
  vector< float > vstOcculusionY;

  vstOcculusion.clear();
  vstOcculusionX.clear();
  vstOcculusionY.clear();

  int iXY = 50;

  for (float x = 100; x < 4500;)
  {
    for (float y = 100; y < 4000;)
    {
      LINE e;

      if (x == 2000)  ////3000
      {
        vstOcclusion.clear();
        vstLaser.clear();

        // RD-LD
        e.fX0 = x + iXY;
        e.fY0 = y - iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        float theta1 = atan(e.fY0 / (e.fX0 - fLrf_set_x)) * RAD2DEG;
        float theta2 = 180 - atan(e.fY1 / (fLrf_set_x - e.fX1)) * RAD2DEG;

        vstOcclusion.push_back(e);

        // LU-LD
        e.fX0 = x - iXY;
        e.fY0 = y + iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        for (int k = (int)(4 * theta1); k <= (int)(4 * theta2); k++)
        {
          stLaser.fX0 = fLrf_set_x;
          stLaser.fY0 = fLrf_set_y;
          stLaser.fX1 = fss_tf_data.fX2[k];
          stLaser.fY1 = fss_tf_data.fY2[k];
          vstLaser.push_back(stLaser);
        }

        for (int k = 0; k <= (int)fss_tf_data.bIsReflect.size(); k++)
        {
          if (fss_tf_data.bIsReflect[k] == true)
          {
            stLaser.fX0 = fLrf_set_x;
            stLaser.fY0 = fLrf_set_y;
            stLaser.fX1 = fss_tf_data.fX1[k];
            stLaser.fY1 = fss_tf_data.fY1[k];
            vstLaser.push_back(stLaser);

            stLaser.fX0 = fss_tf_data.fX1[k];
            stLaser.fY0 = fss_tf_data.fY1[k];
            stLaser.fX1 = fss_tf_data.fX2[k];
            stLaser.fY1 = fss_tf_data.fY2[k];
            vstLaser.push_back(stLaser);
          }
        }

        bool bIs_intersect = is_intersect(vstOcclusion, vstLaser);

        if (bIs_intersect == false)
        {
          vstOcculusionX.push_back(x);
          vstOcculusionY.push_back(y);
        }
      }
      else if (x > 2000)  ////3000
      {
        vstOcclusion.clear();
        vstLaser.clear();

        e.fX0 = x + iXY;
        e.fY0 = y - iXY;
        e.fX1 = x - iXY;
        e.fY1 = y + iXY;
        float theta1 = atan(e.fY0 / (e.fX0 - fLrf_set_x)) * RAD2DEG;
        float theta2 = atan(e.fY1 / (e.fX1 - fLrf_set_x)) * RAD2DEG;

        // RD-LD
        e.fX0 = x + iXY;
        e.fY0 = y - iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        // LU-LD
        e.fX0 = x - iXY;
        e.fY0 = y + iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        for (int k = (int)(4 * theta1); k <= (int)(4 * theta2); k++)
        {
          stLaser.fX0 = fLrf_set_x;
          stLaser.fY0 = fLrf_set_y;
          stLaser.fX1 = fss_tf_data.fX2[k];
          stLaser.fY1 = fss_tf_data.fY2[k];
          vstLaser.push_back(stLaser);
        }

        for (int k = 0; k <= (int)fss_tf_data.bIsReflect.size(); k++)
        {
          if (fss_tf_data.bIsReflect[k] == true)
          {
            stLaser.fX0 = fLrf_set_x;
            stLaser.fY0 = fLrf_set_y;
            stLaser.fX1 = fss_tf_data.fX1[k];
            stLaser.fY1 = fss_tf_data.fY1[k];
            vstLaser.push_back(stLaser);

            stLaser.fX0 = fss_tf_data.fX1[k];
            stLaser.fY0 = fss_tf_data.fY1[k];
            stLaser.fX1 = fss_tf_data.fX2[k];
            stLaser.fY1 = fss_tf_data.fY2[k];
            vstLaser.push_back(stLaser);
          }
        }

        bool bIs_intersect = is_intersect(vstOcclusion, vstLaser);

        if (bIs_intersect == false)
        {
          vstOcculusionX.push_back(x);
          vstOcculusionY.push_back(y);
        }
      }
      else if (x < 2000)  ////3000
      {
        vstOcclusion.clear();
        vstLaser.clear();

        e.fX0 = x + iXY;
        e.fY0 = y + iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        float theta1 = 180 - atan(e.fY0 / (fLrf_set_x - e.fX0)) * RAD2DEG;
        float theta2 = 180 - atan(e.fY1 / (fLrf_set_x - e.fX1)) * RAD2DEG;

        // RU-RD
        e.fX0 = x + iXY;
        e.fY0 = y + iXY;
        e.fX1 = x + iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        // LD-RD
        e.fX0 = x - iXY;
        e.fY0 = y - iXY;
        e.fX1 = x + iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        // LU-LD
        e.fX0 = x - iXY;
        e.fY0 = y + iXY;
        e.fX1 = x - iXY;
        e.fY1 = y - iXY;
        vstOcclusion.push_back(e);

        for (int k = (int)(4 * theta1); k <= (int)(4 * theta2); k++)
        {
          stLaser.fX0 = fLrf_set_x;
          stLaser.fY0 = fLrf_set_y;
          stLaser.fX1 = fss_tf_data.fX2[k];
          stLaser.fY1 = fss_tf_data.fY2[k];
          vstLaser.push_back(stLaser);
        }

        for (int k = 0; k <= (int)fss_tf_data.bIsReflect.size(); k++)
        {
          if (fss_tf_data.bIsReflect[k] == true)
          {
            stLaser.fX0 = fLrf_set_x;
            stLaser.fY0 = fLrf_set_y;
            stLaser.fX1 = fss_tf_data.fX1[k];
            stLaser.fY1 = fss_tf_data.fY1[k];
            vstLaser.push_back(stLaser);

            stLaser.fX0 = fss_tf_data.fX1[k];
            stLaser.fY0 = fss_tf_data.fY1[k];
            stLaser.fX1 = fss_tf_data.fX2[k];
            stLaser.fY1 = fss_tf_data.fY2[k];
            vstLaser.push_back(stLaser);
          }
        }

        bool bIs_intersect = is_intersect(vstOcclusion, vstLaser);

        if (bIs_intersect == false)
        {
          vstOcculusionX.push_back(x);
          vstOcculusionY.push_back(y);
        }
      }
      y = y + iXY * 2;
    }
    x = x + iXY * 2;
  }

  //--------------------------------------------------------------------------
  fss_tf_data.fOcclusionX = vstOcculusionX;
  fss_tf_data.fOcclusionY = vstOcculusionY;

  //--------------------------------------------------------------------------
  rosPub.publish(fss_tf_data);

  //--------------------------------------------------------------------------
  vstLrfDataTemp.clear();

  vstIsForwardPoint.clear();
  vstReflect.clear();
  vstDistance.clear();
  vstIntensity.clear();
  vstIntrinsicIntensity.clear();
  vstAcuteAngle.clear();
  vstX1.clear();
  vstY1.clear();
  vstX2.clear();
  vstY2.clear();

  //--------------------------------------------------------------------------
  double ExeTime = ros::Time::now().toSec() - StartTime;
  printf("ExeTime = %f sec\n", ExeTime);
  ROS_INFO_STREAM("completed sending of fss_tf_data #e");
}

//------------------------------------------------------------------------------
void lrfCallback1(const tms_msg_ss::fss_pre_data::ConstPtr& msg)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed sending of fss_tf_data1 #s");
  double StartTime = ros::Time::now().toSec();

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_tf_data fss_tf_data;

  LRF_DATA stLrfData;
  vector< LRF_DATA > vstLrfDataTemp;

  POINT stPoint;

  float fAngle = 0;
  float fLrf_set_x = 5500;  ////3000
  float fLrf_set_y = 500;

  LINE stMirror_line;
  LINE stLrf_Laser_line;
  POINT stIntersection_point;

  vector< uint8_t > vstReflect;
  vector< uint8_t > vstIsForwardPoint;
  vector< float > vstDistance;
  vector< float > vstIntensity;
  vector< float > vstIntrinsicIntensity;
  vector< float > vstAcuteAngle;
  vector< float > vstX1;
  vector< float > vstY1;
  vector< float > vstX2;
  vector< float > vstY2;

  unsigned int iLength = 0;

  //--------------------------------------------------------------------------
  // Make BackgroundData
  if (iBackgroundCount < 50)
  {
    iLength = msg->fDistance.size();
    for (unsigned int i = 0; i < iLength; i++)
    {
      fBackgroundData[i] = max(fBackgroundData[i], msg->fDistance[i]);
    }

    iBackgroundCount++;
    return;
  }

  //--------------------------------------------------------------------------
  // Make  bIsForwardPoint, bIsReflect, X1, Y1, X2, Y2
  iLength = msg->fDistance.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    if (msg->fDistance[i] + fBackgroundThreshold < fBackgroundData[i])
    {
      stLrfData.bIsForwardPoint = true;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }
    else
    {
      stLrfData.bIsForwardPoint = false;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }

    //----------------------------------------------------------------------
    fAngle = i * 0.36 * DEG2RAD;

    stMirror_line.fX0 = 0;
    stMirror_line.fY0 = 0;
    stMirror_line.fX1 = 0;
    stMirror_line.fY1 = 4000;

    stPoint.fX = stLrfData.fDistance * cos(fAngle) + fLrf_set_x;
    stPoint.fY = stLrfData.fDistance * sin(fAngle) + fLrf_set_y;

    stLrf_Laser_line.fX0 = fLrf_set_x;
    stLrf_Laser_line.fY0 = fLrf_set_y;
    stLrf_Laser_line.fX1 = stPoint.fX;
    stLrf_Laser_line.fY1 = stPoint.fY;

    //----------------------------------------------------------------------
    stLrfData.bIsReflect = false;  // get_point_of_intersection(stMirror_line, stLrf_Laser_line, &stIntersection_point);

    stLrfData.stCoordinate.fX1 = stPoint.fX;
    stLrfData.stCoordinate.fY1 = stPoint.fY;
    stLrfData.stCoordinate.fX2 = stPoint.fX;
    stLrfData.stCoordinate.fY2 = stPoint.fY;

    vstLrfDataTemp.push_back(stLrfData);

    //----------------------------------------------------------------------
    vstReflect.push_back(stLrfData.bIsReflect);
    vstIsForwardPoint.push_back(stLrfData.bIsForwardPoint);
    vstDistance.push_back(stLrfData.fDistance);
    vstIntensity.push_back(stLrfData.fIntensity);
    vstX1.push_back(stLrfData.stCoordinate.fX1);
    vstY1.push_back(stLrfData.stCoordinate.fY1);
    vstX2.push_back(stLrfData.stCoordinate.fX2);
    vstY2.push_back(stLrfData.stCoordinate.fY2);
  }

  //--------------------------------------------------------------------------
  fss_tf_data.header.frame_id = "fss_tf_data1";
  fss_tf_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_tf_data.tMeasuredTime = msg->tMeasuredTime;

  fss_tf_data.bIsReflect = vstReflect;
  fss_tf_data.bIsForwardPoint = vstIsForwardPoint;
  fss_tf_data.fDistance = vstDistance;
  fss_tf_data.fIntensity = vstIntensity;
  fss_tf_data.fIntrinsicIntensity = vstIntrinsicIntensity;
  fss_tf_data.fAcuteAngle = vstAcuteAngle;
  fss_tf_data.fX1 = vstX1;
  fss_tf_data.fY1 = vstY1;
  fss_tf_data.fX2 = vstX2;
  fss_tf_data.fY2 = vstY2;
  fss_tf_data.fY2 = vstY2;

  //--------------------------------------------------------------------------
  rosPub1.publish(fss_tf_data);

  //--------------------------------------------------------------------------
  vstLrfDataTemp.clear();

  vstIsForwardPoint.clear();
  vstReflect.clear();
  vstDistance.clear();
  vstIntensity.clear();
  vstIntrinsicIntensity.clear();
  vstAcuteAngle.clear();
  vstX1.clear();
  vstY1.clear();
  vstX2.clear();
  vstY2.clear();

  //--------------------------------------------------------------------------
  double ExeTime = ros::Time::now().toSec() - StartTime;
  printf("ExeTime = %f sec\n", ExeTime);
  ROS_INFO_STREAM("completed sending of fss_tf_data1 #e");
}

//------------------------------------------------------------------------------
void lrfCallback2(const tms_msg_ss::fss_pre_data::ConstPtr& msg)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed sending of fss_tf_data2 #s");
  double StartTime = ros::Time::now().toSec();

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_tf_data fss_tf_data;

  LRF_DATA stLrfData;
  vector< LRF_DATA > vstLrfDataTemp;

  POINT stPoint;

  float fAngle = 0;
  float fLrf_set_x = 6000;
  float fLrf_set_y = 4000;

  LINE stMirror_line;
  LINE stLrf_Laser_line;
  POINT stIntersection_point;

  vector< uint8_t > vstReflect;
  vector< uint8_t > vstIsForwardPoint;
  vector< float > vstDistance;
  vector< float > vstIntensity;
  vector< float > vstIntrinsicIntensity;
  vector< float > vstAcuteAngle;
  vector< float > vstX1;
  vector< float > vstY1;
  vector< float > vstX2;
  vector< float > vstY2;

  unsigned int iLength = 0;

  //--------------------------------------------------------------------------
  // Make BackgroundData
  if (iBackgroundCount < 50)
  {
    iLength = msg->fDistance.size();
    for (unsigned int i = 0; i < iLength; i++)
    {
      fBackgroundData[i] = max(fBackgroundData[i], msg->fDistance[i]);
    }

    iBackgroundCount++;
    return;
  }

  //--------------------------------------------------------------------------
  // Make  bIsForwardPoint, bIsReflect, X1, Y1, X2, Y2
  iLength = msg->fDistance.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    if (msg->fDistance[i] + fBackgroundThreshold < fBackgroundData[i])
    {
      stLrfData.bIsForwardPoint = true;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }
    else
    {
      stLrfData.bIsForwardPoint = false;
      stLrfData.fDistance = msg->fDistance[i];
      stLrfData.fIntensity = msg->fIntensity[i];
    }

    //----------------------------------------------------------------------
    fAngle = i * 0.36 * DEG2RAD;

    stMirror_line.fX0 = 0;
    stMirror_line.fY0 = 0;
    stMirror_line.fX1 = 0;
    stMirror_line.fY1 = 4000;

    stPoint.fX = stLrfData.fDistance * cos(fAngle) + fLrf_set_x;
    stPoint.fY = stLrfData.fDistance * sin(fAngle) + fLrf_set_y;

    stLrf_Laser_line.fX0 = fLrf_set_x;
    stLrf_Laser_line.fY0 = fLrf_set_y;
    stLrf_Laser_line.fX1 = stPoint.fX;
    stLrf_Laser_line.fY1 = stPoint.fY;

    //----------------------------------------------------------------------
    stLrfData.bIsReflect = false;  // get_point_of_intersection(stMirror_line, stLrf_Laser_line, &stIntersection_point);

    stLrfData.stCoordinate.fX1 = stPoint.fX;
    stLrfData.stCoordinate.fY1 = stPoint.fY;
    stLrfData.stCoordinate.fX2 = stPoint.fX;
    stLrfData.stCoordinate.fY2 = stPoint.fY;

    vstLrfDataTemp.push_back(stLrfData);

    //----------------------------------------------------------------------
    vstReflect.push_back(stLrfData.bIsReflect);
    vstIsForwardPoint.push_back(stLrfData.bIsForwardPoint);
    vstDistance.push_back(stLrfData.fDistance);
    vstIntensity.push_back(stLrfData.fIntensity);
    vstX1.push_back(stLrfData.stCoordinate.fX1);
    vstY1.push_back(stLrfData.stCoordinate.fY1);
    vstX2.push_back(stLrfData.stCoordinate.fX2);
    vstY2.push_back(stLrfData.stCoordinate.fY2);
  }

  //--------------------------------------------------------------------------
  fss_tf_data.header.frame_id = "fss_tf_data2";
  fss_tf_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_tf_data.tMeasuredTime = msg->tMeasuredTime;

  fss_tf_data.bIsReflect = vstReflect;
  fss_tf_data.bIsForwardPoint = vstIsForwardPoint;
  fss_tf_data.fDistance = vstDistance;
  fss_tf_data.fIntensity = vstIntensity;
  fss_tf_data.fIntrinsicIntensity = vstIntrinsicIntensity;
  fss_tf_data.fAcuteAngle = vstAcuteAngle;
  fss_tf_data.fX1 = vstX1;
  fss_tf_data.fY1 = vstY1;
  fss_tf_data.fX2 = vstX2;
  fss_tf_data.fY2 = vstY2;
  fss_tf_data.fY2 = vstY2;

  //--------------------------------------------------------------------------
  rosPub2.publish(fss_tf_data);

  //--------------------------------------------------------------------------
  vstLrfDataTemp.clear();

  vstIsForwardPoint.clear();
  vstReflect.clear();
  vstDistance.clear();
  vstIntensity.clear();
  vstIntrinsicIntensity.clear();
  vstAcuteAngle.clear();
  vstX1.clear();
  vstY1.clear();
  vstX2.clear();
  vstY2.clear();

  //--------------------------------------------------------------------------
  double ExeTime = ros::Time::now().toSec() - StartTime;
  printf("ExeTime = %f sec\n", ExeTime);
  ROS_INFO_STREAM("completed sending of fss_tf_data2 #e");
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_transform");
  ros::NodeHandle nh;
  rosSub = nh.subscribe("fss_pre_data", 10, lrfCallback);
  rosPub = nh.advertise< tms_msg_ss::fss_tf_data >("fss_tf_data", 10);

  rosSub1 = nh.subscribe("fss_pre_data1", 10, lrfCallback1);
  rosPub1 = nh.advertise< tms_msg_ss::fss_tf_data >("fss_tf_data1", 10);

  rosSub2 = nh.subscribe("fss_pre_data2", 10, lrfCallback2);
  rosPub2 = nh.advertise< tms_msg_ss::fss_tf_data >("fss_tf_data2", 10);
  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}
