//------------------------------------------------------------------------------
// @file   : fss_class_data.cpp
// @brief  : Make class data
// @author : Yoonseok Pyo
// @version: Ver0.8 (since 2012.06.01)
// @date   : 2012.09.04
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/date_time/local_time/local_time.hpp>

#include <tms_ss_fss_exe/common.h>

#include <tms_msg_ss/fss_tf_datas.h>
#include <tms_msg_ss/fss_cluster_data.h>
#include <tms_msg_ss/fss_class_data.h>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;
ros::Publisher rosPub;

tms_msg_ss::fss_cluster_data lrf_cluster_before;
tms_msg_ss::fss_cluster_data cluster;

void rosCheckTime(boost::posix_time::ptime time)
{
  struct tm T = boost::posix_time::to_tm(time);  // ptime -> struct tm

  cout << T.tm_year + 1900 << "/" << T.tm_mon + 1 << "/" << T.tm_mday << " " << T.tm_hour << ":" << T.tm_min << ":"
       << T.tm_sec << endl;

  // or save in TMS
}

//------------------------------------------------------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void lrfCallback(const tms_msg_ss::fss_cluster_data::ConstPtr &msg)
{
  ROS_INFO_STREAM("completed sending of fss_class_data #s");
  tms_msg_ss::fss_cluster_data fss_class_data;
  tms_msg_ss::fss_tf_datas lrf_data;

  vector< uint32_t > vstScanID;
  vector< uint8_t > vstReflect;
  vector< uint8_t > vstIsForwardPoint;
  vector< float > vstDistance;
  vector< float > vstIntensity;
  vector< float > fIntrinsicIntensity;
  vector< float > vstAcuteAngle;
  vector< float > vstX1;
  vector< float > vstY1;
  vector< float > vstX2;
  vector< float > vstY2;

  bool iLength = lrf_cluster_before.iID.empty();

  if (iLength == true)
  {
    lrf_cluster_before = *msg;
  }
  else
  {
    cluster = *msg;

    for (unsigned int i = 0; i < cluster.iID.size(); i++)
    {
      if (cluster.fAvgIntrinsicIntensity[i] >= 5000 && cluster.fSize[i] < 250)
      {
        fss_class_data.iID.push_back(23);  // chair-1
      }
      else if (cluster.fAvgIntrinsicIntensity[i] > 3100 && cluster.fAvgIntrinsicIntensity[i] < 5000 &&
               cluster.fSize[i] < 120)
      {
        fss_class_data.iID.push_back(21);  // wagon-1
      }
      else if (cluster.fSize[i] < 450 && cluster.fSize[i] >= 250 && cluster.fAvgIntrinsicIntensity[i] > 4000)
      {
        fss_class_data.iID.push_back(2);  // roomba
      }
      else if (cluster.fSize[i] > 500 && cluster.fAvgIntrinsicIntensity[i] > 4000)
      {
        fss_class_data.iID.push_back(1);  // smartpal
      }
      else if (cluster.fAvgIntrinsicIntensity[i] > 4000)
      {
        fss_class_data.iID.push_back(cluster.iID[i] + 301);
      }
      else
      {
        fss_class_data.iID.push_back(cluster.iID[i] + 201);
      }

      fss_class_data.fAvgIntrinsicIntensity.push_back(cluster.fAvgIntrinsicIntensity[i]);
      fss_class_data.fCenterX.push_back(cluster.fCenterX[i]);
      fss_class_data.fCenterY.push_back(cluster.fCenterY[i]);
      fss_class_data.fSize.push_back(cluster.fSize[i]);

      vstScanID.clear();
      vstReflect.clear();
      vstIsForwardPoint.clear();
      vstDistance.clear();
      vstIntensity.clear();
      fIntrinsicIntensity.clear();
      vstAcuteAngle.clear();
      vstX1.clear();
      vstY1.clear();
      vstX2.clear();
      vstY2.clear();

      for (unsigned int k = 0; k < cluster.LrfData[i].bIsReflect.size(); k++)
      {
        vstScanID.push_back(cluster.LrfData[i].iScanID[k]);
        vstReflect.push_back(cluster.LrfData[i].bIsReflect[k]);
        vstIsForwardPoint.push_back(cluster.LrfData[i].bIsForwardPoint[k]);
        vstDistance.push_back(cluster.LrfData[i].fDistance[k]);
        vstIntensity.push_back(cluster.LrfData[i].fIntensity[k]);
        fIntrinsicIntensity.push_back(cluster.LrfData[i].fIntrinsicIntensity[k]);
        vstAcuteAngle.push_back(cluster.LrfData[i].fAcuteAngle[k]);
        vstX1.push_back(cluster.LrfData[i].fX1[k]);
        vstY1.push_back(cluster.LrfData[i].fY1[k]);
        vstX2.push_back(cluster.LrfData[i].fX2[k]);
        vstY2.push_back(cluster.LrfData[i].fY2[k]);
      }
      lrf_data.iScanID = vstScanID;
      lrf_data.bIsReflect = vstReflect;
      lrf_data.bIsForwardPoint = vstIsForwardPoint;
      lrf_data.fDistance = vstDistance;
      lrf_data.fIntensity = vstIntensity;
      lrf_data.fIntrinsicIntensity = fIntrinsicIntensity;
      lrf_data.fAcuteAngle = vstAcuteAngle;
      lrf_data.fX1 = vstX1;
      lrf_data.fY1 = vstY1;
      lrf_data.fX2 = vstX2;
      lrf_data.fY2 = vstY2;

      fss_class_data.LrfData.push_back(lrf_data);
    }

    fss_class_data.header.frame_id = "fss_class_data";
    fss_class_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_class_data.tMeasuredTime = cluster.tMeasuredTime;

    fss_class_data.fOcclusionX = cluster.fOcclusionX;
    fss_class_data.fOcclusionY = cluster.fOcclusionY;
    rosPub.publish(fss_class_data);

    lrf_cluster_before.iID.clear();
    lrf_cluster_before.fCenterX.clear();
    lrf_cluster_before.fCenterY.clear();
    lrf_cluster_before.fSize.clear();
    lrf_cluster_before.fAvgIntrinsicIntensity.clear();
    lrf_cluster_before.LrfData.clear();
    lrf_cluster_before.fOcclusionX.clear();
    lrf_cluster_before.fOcclusionY.clear();

    lrf_cluster_before = fss_class_data;
  }
  ROS_INFO_STREAM("completed sending of fss_class_data #e");
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_classify");
  ros::NodeHandle nh;
  rosSub = nh.subscribe("fss_cluster_data", 10, lrfCallback);
  rosPub = nh.advertise< tms_msg_ss::fss_class_data >("fss_class_data", 10);

  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}
