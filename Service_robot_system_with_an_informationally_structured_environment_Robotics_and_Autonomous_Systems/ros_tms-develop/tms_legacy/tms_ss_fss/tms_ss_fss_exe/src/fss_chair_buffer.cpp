//------------------------------------------------------------------------------
// @file   : fss_chair_buffer.cpp
// @brief  : accumulate chair position from fss_chair_tracking
// @author : Masahide Tanaka
// @version: Ver0.1 (since 2012.10.23)
// @date   : 2012.10.23
//------------------------------------------------------------------------------

#include <time.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include <list>
#include <tms_msg_ss/fss_object_data.h>
#include <tms_msg_ss/fss_observed_datas.h>
#include <fstream>
#include <sstream>

// for furniture movement
typedef struct
{
  ros::Time tStartTime;
  ros::Time tEndTime;
  float fCenterX;
  float fCenterY;
} FuniturePosition;

//------------------------------------------------------------------------------
using namespace std;

ros::Subscriber rosChairSub;
ros::Publisher rosChairPub;

std::vector< FuniturePosition > chairPos;

float getLength(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void callback(const tms_msg_ss::fss_object_data::ConstPtr &msg)
{
  bool isPush = false;

  float fCenterX = msg->msgTMSInfo[0].fX;
  float fCenterY = msg->msgTMSInfo[0].fY;

  if ((fCenterX == 0.0 && fCenterY == 4000.0) || (fCenterX == 0.0 && fCenterY == 0.0))
  {
    // update eTime
    if (chairPos.size() > 0)
      chairPos.back().tEndTime = msg->tMeasuredTime;
  }
  else
  {
    if (chairPos.size() == 0)
    {
      FuniturePosition tempFuniturePos;
      tempFuniturePos.tStartTime = msg->tMeasuredTime;
      tempFuniturePos.tEndTime = msg->tMeasuredTime;
      tempFuniturePos.fCenterX = fCenterX;
      tempFuniturePos.fCenterY = fCenterY;
      chairPos.push_back(tempFuniturePos);
      isPush = true;
    }
    else
    {
      float _fCenterX = chairPos.back().fCenterX;
      float _fCenterY = chairPos.back().fCenterY;

      float len = getLength(fCenterX, fCenterY, _fCenterX, _fCenterY);

      if (len >= 50)
      {
        FuniturePosition tempFuniturePos;
        tempFuniturePos.tStartTime = msg->tMeasuredTime;
        tempFuniturePos.tEndTime = msg->tMeasuredTime;
        tempFuniturePos.fCenterX = fCenterX;
        tempFuniturePos.fCenterY = fCenterY;
        chairPos.push_back(tempFuniturePos);
        isPush = true;
      }
      else
      {
        // update eTime
        chairPos.back().tEndTime = msg->tMeasuredTime;
      }
    }
  }

  //------------------------------------------------------------------------------
  // publish
  tms_msg_ss::fss_observed_datas fss_chair_buffer_data;
  for (unsigned int i = 0; i < chairPos.size(); i++)
  {
    fss_chair_buffer_data.tStartTime.push_back(chairPos[i].tStartTime);
    fss_chair_buffer_data.tEndTime.push_back(chairPos[i].tEndTime);
    fss_chair_buffer_data.fCenterX.push_back(chairPos[i].fCenterX);
    fss_chair_buffer_data.fCenterY.push_back(chairPos[i].fCenterY);
  }
  rosChairPub.publish(fss_chair_buffer_data);

  if (isPush)
    printf("ChairPos[%ld]: (%f, %f)\n", chairPos.size(), chairPos.back().fCenterX, chairPos.back().fCenterY);
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fss_chair_buffer");
  ros::NodeHandle nh;
  rosChairSub = nh.subscribe("fss_chair_data", 10, callback);
  rosChairPub = nh.advertise< tms_msg_ss::fss_observed_datas >("fss_chair_buffer_data", 10);

  ros::spin();

  return (0);
}
