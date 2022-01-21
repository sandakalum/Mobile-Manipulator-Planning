//------------------------------------------------------------------------------
// @file   : fss_preprocess.cpp
// @brief  : Median value and noise filter
// @author : Yoonseok Pyo
// @version: Ver0.5 (since 2012.06.01)
// @date   : 2012.07.13
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tms_ss_fss_exe/common.h>
#include <tms_msg_ss/fss_pre_data.h>
#include <cmath>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;
ros::Publisher rosPub;
ros::Subscriber rosSub1;
ros::Publisher rosPub1;
ros::Subscriber rosSub2;
ros::Publisher rosPub2;

std::list< QUEUE > lstUtm30LX_raw_data_average;
std::list< QUEUE > lstUtm30LX_raw_data_average1;
std::list< QUEUE > lstUtm30LX_raw_data_average2;

vector< float > utm30lx_raw_distance_before;
vector< float > utm30lx_raw_intensity_before;

int iLrf_scan_max_count = 721;

//------------------------------------------------------------------------------
float getMedian(vector< float > array, size_t arraySize)
{
  size_t center = arraySize / 2;
  if (arraySize % 2 == 1)
  {
    return array[center];
  }
  else
  {
    return (array[center - 1] + array[center]) / 2.0;
  }
}

//----------------------------------------------d--------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void lrfCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_STREAM("completed sending of fss_pre_data #s");
  //--------------------------------------------------------------------------
  tms_msg_ss::fss_pre_data fss_pre_data;
  list< QUEUE >::iterator it1;
  vector< LRF >::iterator it2;
  vector< float > utm30lx_raw_distance;
  vector< float > utm30lx_raw_intensity;
  QUEUE stTempBackup;
  LRF stTmepData;
  unsigned int iLength = 0;

  float fLrf_set_x = 2000;
  ////float fLrf_set_x = 3000;
  // float fLrf_set_y = 0;

  vector< float > raw_distance[iLrf_scan_max_count];
  vector< float > raw_intensity[iLrf_scan_max_count];

  ros::Time tNow = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;

  //--------------------------------------------------------------------------
  stTempBackup.vstRawDataQueue.clear();
  utm30lx_raw_distance.clear();
  utm30lx_raw_intensity.clear();

  for (int i = 0; i < iLrf_scan_max_count; i++)
  {
    raw_distance[i].clear();
    raw_intensity[i].clear();
  }

  //--------------------------------------------------------------------------
  iLength = msg->ranges.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    stTmepData.fDistance = msg->ranges[iLength - 1 - i] * 1000;  // meter
    stTmepData.fIntensity = msg->intensities[iLength - 1 - i];

    // LRF noise
    if (i < 10 || i > 710)
    {
      stTmepData.fDistance = 0;
      stTmepData.fIntensity = 0;
    }

    // LRF Reflectance rate 0~10degree
    if (i >= 680 && i <= 710)
    {
      float fAngle = (180 - i * 0.25) * DEG2RAD;
      float fMirrorDistance = fLrf_set_x / cos(fAngle) + 1;  // offset 1mm

      if (stTmepData.fDistance > fMirrorDistance)
      {
        stTmepData.fDistance = fMirrorDistance;
      }
    }
    stTempBackup.vstRawDataQueue.push_back(stTmepData);
  }

  //--------------------------------------------------------------------------
  lstUtm30LX_raw_data_average.push_back(stTempBackup);
  stTempBackup.vstRawDataQueue.clear();

  //--------------------------------------------------------------------------
  for (int i = 0; i < iLrf_scan_max_count; i++)
  {
    for (it1 = lstUtm30LX_raw_data_average.begin(); it1 != lstUtm30LX_raw_data_average.end(); it1++)
    {
      it2 = it1->vstRawDataQueue.begin();

      raw_distance[i].push_back(it2[i].fDistance);
      raw_intensity[i].push_back(it2[i].fIntensity);
    }
  }

  //--------------------------------------------------------------------------
  for (int i = 0; i < iLrf_scan_max_count; i++)
  {
    sort(raw_distance[i].begin(), raw_distance[i].end());
    stTmepData.fDistance = getMedian(raw_distance[i], raw_distance[i].size());

    sort(raw_intensity[i].begin(), raw_intensity[i].end());
    stTmepData.fIntensity = getMedian(raw_intensity[i], raw_intensity[i].size());

    utm30lx_raw_distance.push_back(stTmepData.fDistance);
    utm30lx_raw_intensity.push_back(stTmepData.fIntensity);
  }

  //--------------------------------------------------------------------------
  if (lstUtm30LX_raw_data_average.size() >= 5)  // 0.25sec = 0.025*10
  {
    it1 = lstUtm30LX_raw_data_average.begin();
    lstUtm30LX_raw_data_average.erase(it1);
  }

  fss_pre_data.header.frame_id = "fss_pre_data";
  fss_pre_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_pre_data.tMeasuredTime = tNow;
  fss_pre_data.fDistance = utm30lx_raw_distance;
  fss_pre_data.fIntensity = utm30lx_raw_intensity;
  rosPub.publish(fss_pre_data);
  ROS_INFO_STREAM("completed sending of fss_pre_data #e");
}

//------------------------------------------------------------------------------
void lrfCallback1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_STREAM("completed sending of fss_pre_data1 #s");
  //--------------------------------------------------------------------------
  tms_msg_ss::fss_pre_data fss_pre_data;
  vector< float > utm30lx_raw_distance;
  vector< float > utm30lx_raw_intensity;
  LRF stTmepData;
  unsigned int iLength = 0;
  ros::Time tNow = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  utm30lx_raw_distance.clear();
  utm30lx_raw_intensity.clear();

  //--------------------------------------------------------------------------
  iLength = msg->ranges.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    stTmepData.fDistance = msg->ranges[iLength - 1 - i] * 1000;  // meter -> mm
    stTmepData.fIntensity = 0;

    if (!(stTmepData.fDistance > 0 && stTmepData.fDistance < 4000))
      stTmepData.fDistance = 0;

    utm30lx_raw_distance.push_back(stTmepData.fDistance);
    utm30lx_raw_intensity.push_back(stTmepData.fIntensity);
  }

  //--------------------------------------------------------------------------
  fss_pre_data.header.frame_id = "fss_pre_data1";
  fss_pre_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_pre_data.tMeasuredTime = tNow;
  fss_pre_data.fDistance = utm30lx_raw_distance;
  fss_pre_data.fIntensity = utm30lx_raw_intensity;
  rosPub1.publish(fss_pre_data);
  ROS_INFO_STREAM("completed sending of fss_pre_data1 #e");
}

//------------------------------------------------------------------------------
void lrfCallback2(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_STREAM("completed sending of fss_pre_data2 #s");
  //--------------------------------------------------------------------------
  tms_msg_ss::fss_pre_data fss_pre_data;
  vector< float > utm30lx_raw_distance;
  vector< float > utm30lx_raw_intensity;
  LRF stTmepData;
  unsigned int iLength = 0;
  ros::Time tNow = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  utm30lx_raw_distance.clear();
  utm30lx_raw_intensity.clear();

  //--------------------------------------------------------------------------
  iLength = msg->ranges.size();
  for (unsigned int i = 0; i < iLength; i++)
  {
    stTmepData.fDistance = msg->ranges[iLength - 1 - i] * 1000;  // meter -> mm
    stTmepData.fIntensity = 0;

    if (!(stTmepData.fDistance > 0 && stTmepData.fDistance < 10000))
      stTmepData.fDistance = 0;

    utm30lx_raw_distance.push_back(stTmepData.fDistance);
    utm30lx_raw_intensity.push_back(stTmepData.fIntensity);
  }

  //--------------------------------------------------------------------------
  fss_pre_data.header.frame_id = "fss_pre_data2";
  fss_pre_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_pre_data.tMeasuredTime = tNow;
  fss_pre_data.fDistance = utm30lx_raw_distance;
  fss_pre_data.fIntensity = utm30lx_raw_intensity;
  rosPub2.publish(fss_pre_data);
  ROS_INFO_STREAM("completed sending of fss_pre_data2 #e");
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_preprocess");
  ros::NodeHandle nh;
  rosSub = nh.subscribe("fss_raw_data", 10, lrfCallback);
  rosPub = nh.advertise< tms_msg_ss::fss_pre_data >("fss_pre_data", 10);

  rosSub1 = nh.subscribe("scan1", 10, lrfCallback1);
  rosPub1 = nh.advertise< tms_msg_ss::fss_pre_data >("fss_pre_data1", 10);

  rosSub2 = nh.subscribe("scan2", 10, lrfCallback2);
  rosPub2 = nh.advertise< tms_msg_ss::fss_pre_data >("fss_pre_data2", 10);

  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}

//------------------------------------------------------------------------------
