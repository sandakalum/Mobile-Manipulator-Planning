//------------------------------------------------------------------------------
// @file   : fss_cluster_data.cpp
// @brief  : Make cluster data
// @author : Yoonseok Pyo
// @version: Ver0.7 (since 2012.06.01)
// @date   : 2012.09.27
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tms_ss_fss_exe/common.h>

#include <tms_msg_ss/fss_tf_data.h>
#include <tms_msg_ss/fss_tf_datas.h>
#include <tms_msg_ss/fss_cluster_data.h>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;
ros::Publisher rosPub;

tms_msg_ss::fss_cluster_data fss_cluster_before;
tms_msg_ss::fss_cluster_data fss_cluster_before2;
tms_msg_ss::fss_cluster_data fss_cluster_before3;

int iLrf_scan_max_count = 721;
bool bIsBetweenForwardPoint = false;
bool bUseReflectPoint = true;
bool bUseSoloReflectCluster = true;

//------------------------------------------------------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
void lrfCallback(const tms_msg_ss::fss_tf_data::ConstPtr &msg)
{
  ROS_INFO_STREAM("completed sending of fss_cluster_data #s");

  tms_msg_ss::fss_tf_datas fss_tf_data;
  tms_msg_ss::fss_cluster_data fss_cluster_data;

  bool bIsNear = false;
  float fLen = 0.0;
  float fThreshold = 0.0;  // 2*(m_vstLrf_data[i].dRaw_distance)*M_PI*(0.25/360) + distance accuracy ±30mm;
  unsigned int iGroupID = 0;
  int NOT_NOISE_OBJECT = 4;
  float fCenterX = 0.0;
  float fCenterY = 0.0;
  float fAvgIntrinsicIntensity = 0.0;
  float fMinDistance = 200.0;          // 200mm
  float fMaxStandardDeviation = 35.0;  // 35mm

  vector< uint32_t > vstGroupID;
  vector< float > vstCenterX;
  vector< float > vstCenterY;
  vector< float > vstSize;
  vector< float > vstAvgIntrinsicIntensity;

  vector< uint32_t > vstScanID;
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

  //--------------------------------------------------------------------------
  iGroupID = 2;
  unsigned int iLength = msg->fDistance.size();
  for (unsigned int i = 0; i < iLength - 1; i++)
  {
    //----------------------------------------------------------------------
    if (msg->bIsForwardPoint[i] == false)
    {
      // nothing!
      bIsBetweenForwardPoint = true;
      if ((int)vstScanID.size() >= NOT_NOISE_OBJECT)
      {
        //----------------------------------------------------------
        vstGroupID.push_back(iGroupID);

        //----------------------------------------------------------
        fCenterX = fCenterY = 0.0;
        for (unsigned j = 0; j < vstX2.size(); j++)
        {
          fCenterX += vstX2[j];
          fCenterY += vstY2[j];
        }
        fCenterX = fCenterX / vstX2.size();
        fCenterY = fCenterY / vstY2.size();
        vstCenterX.push_back(fCenterX);
        vstCenterY.push_back(fCenterY);

        //----------------------------------------------------------
        float fMax_size = 0.0;
        float fLength = 0.0;
        for (unsigned j = 0; j < vstX2.size(); j++)
        {
          for (unsigned k = j; k < vstX2.size(); k++)
          {
            fLength = get_length(vstX2[j], vstY2[j], vstX2[k], vstY2[k]);

            if (fLength >= fMax_size)
            {
              fMax_size = fLength;
            }
          }
        }
        vstSize.push_back(fMax_size);

        //----------------------------------------------------------
        fAvgIntrinsicIntensity = 0.0;
        for (unsigned j = 0; j < vstIntrinsicIntensity.size(); j++)
        {
          fAvgIntrinsicIntensity += vstIntrinsicIntensity[j];
        }
        fAvgIntrinsicIntensity = fAvgIntrinsicIntensity / vstIntrinsicIntensity.size();
        vstAvgIntrinsicIntensity.push_back(fAvgIntrinsicIntensity);

        //----------------------------------------------------------
        fss_tf_data.iScanID = vstScanID;
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

        fss_cluster_data.LrfData.push_back(fss_tf_data);
        iGroupID++;
      }

      //--------------------------------------------------------------
      vstScanID.clear();
      vstReflect.clear();
      vstIsForwardPoint.clear();
      vstDistance.clear();
      vstIntensity.clear();
      vstIntrinsicIntensity.clear();
      vstAcuteAngle.clear();
      vstX1.clear();
      vstY1.clear();
      vstX2.clear();
      vstY2.clear();
    }

    //----------------------------------------------------------------------
    if (msg->fDistance[i] < fMinDistance)
    {
      // nothing!
      bIsBetweenForwardPoint = false;
      continue;
    }

    //----------------------------------------------------------------------
    if (msg->fDistance[i] >= fMinDistance)
    {
      float f2DD = 2 * msg->fDistance[i] * msg->fDistance[i];
      float fDistanceAdjacentPoint = sqrt(f2DD - f2DD * 0.99999);  // cos(0.25degree) = 0.99999
      fThreshold = fDistanceAdjacentPoint + fMaxStandardDeviation;
      fLen = abs(msg->fDistance[i] - msg->fDistance[i + 1]);

      bIsNear = (fLen < fThreshold && !bIsBetweenForwardPoint);
      bIsBetweenForwardPoint = false;

      //------------------------------------------------------------------
      if (bIsNear)
      {
        if (msg->fAcuteAngle[i] < 1.3963)
        {
          vstScanID.push_back(i);
          vstReflect.push_back(msg->bIsReflect[i]);
          vstIsForwardPoint.push_back(msg->bIsForwardPoint[i]);
          vstDistance.push_back(msg->fDistance[i]);
          vstIntensity.push_back(msg->fIntensity[i]);
          vstIntrinsicIntensity.push_back(msg->fIntrinsicIntensity[i]);
          vstAcuteAngle.push_back(msg->fAcuteAngle[i]);
          vstX1.push_back(msg->fX1[i]);
          vstY1.push_back(msg->fY1[i]);
          vstX2.push_back(msg->fX2[i]);
          vstY2.push_back(msg->fY2[i]);
        }
      }
      else
      {
        if (msg->fAcuteAngle[i] < 1.3963)
        {
          vstScanID.push_back(i);
          vstReflect.push_back(msg->bIsReflect[i]);
          vstIsForwardPoint.push_back(msg->bIsForwardPoint[i]);
          vstDistance.push_back(msg->fDistance[i]);
          vstIntensity.push_back(msg->fIntensity[i]);
          vstIntrinsicIntensity.push_back(msg->fIntrinsicIntensity[i]);
          vstAcuteAngle.push_back(msg->fAcuteAngle[i]);
          vstX1.push_back(msg->fX1[i]);
          vstY1.push_back(msg->fY1[i]);
          vstX2.push_back(msg->fX2[i]);
          vstY2.push_back(msg->fY2[i]);
        }

        if ((int)vstScanID.size() >= NOT_NOISE_OBJECT)
        {
          //----------------------------------------------------------
          vstGroupID.push_back(iGroupID);

          //----------------------------------------------------------
          fCenterX = fCenterY = 0.0;
          for (unsigned j = 0; j < vstX2.size(); j++)
          {
            fCenterX += vstX2[j];
            fCenterY += vstY2[j];
          }
          fCenterX = fCenterX / vstX2.size();
          fCenterY = fCenterY / vstY2.size();
          vstCenterX.push_back(fCenterX);
          vstCenterY.push_back(fCenterY);

          //----------------------------------------------------------
          float fMax_size = 0.0;
          float fLength = 0.0;
          for (unsigned j = 0; j < vstX2.size(); j++)
          {
            for (unsigned k = j; k < vstX2.size(); k++)
            {
              fLength = get_length(vstX2[j], vstY2[j], vstX2[k], vstY2[k]);

              if (fLength >= fMax_size)
              {
                fMax_size = fLength;
              }
            }
          }
          vstSize.push_back(fMax_size);

          //----------------------------------------------------------
          fAvgIntrinsicIntensity = 0.0;
          for (unsigned j = 0; j < vstIntrinsicIntensity.size(); j++)
          {
            fAvgIntrinsicIntensity += vstIntrinsicIntensity[j];
          }
          fAvgIntrinsicIntensity = fAvgIntrinsicIntensity / vstIntrinsicIntensity.size();
          vstAvgIntrinsicIntensity.push_back(fAvgIntrinsicIntensity);

          //----------------------------------------------------------
          fss_tf_data.iScanID = vstScanID;
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

          fss_cluster_data.LrfData.push_back(fss_tf_data);
          iGroupID++;
        }

        //--------------------------------------------------------------
        vstScanID.clear();
        vstReflect.clear();
        vstIsForwardPoint.clear();
        vstDistance.clear();
        vstIntensity.clear();
        vstIntrinsicIntensity.clear();
        vstAcuteAngle.clear();
        vstX1.clear();
        vstY1.clear();
        vstX2.clear();
        vstY2.clear();
      }
    }
  }

  //--------------------------------------------------------------------------
  if (bUseReflectPoint == true)
  {
    float fLengthBetweenGroups = 0.0;
    float fReflectGroupCircleRadius = 0.0;
    float fDirectGroupCircleRadius = 0.0;
    float fCircleRadiusOffset = 0.0;

    iLength = fss_cluster_data.LrfData.size();
    for (unsigned int i = 0; i < iLength; i++)
    {
      printf("Group Num = %d\n", iLength);

      if (fss_cluster_data.LrfData[i].bIsReflect[0] == true)  // Reflect Group
      {
        printf("Reflect Group = %d\n", i);

        for (unsigned int j = 0; j < iLength; j++)
        {
          if (fss_cluster_data.LrfData[j].bIsReflect[0] == false && i != j)  // Direct Group
          {
            printf("Direct Group = %d\n", j);

            fReflectGroupCircleRadius = vstSize[i] / 2 + fCircleRadiusOffset;
            fDirectGroupCircleRadius = vstSize[j] / 2 + fCircleRadiusOffset;
            fLengthBetweenGroups = get_length(vstCenterX[i], vstCenterY[i], vstCenterX[j], vstCenterY[j]);

            printf("fReflectGroupCircleRadius = %f\n", fReflectGroupCircleRadius);
            printf("fDirectGroupCircleRadius  = %f\n", fDirectGroupCircleRadius);
            printf("fLengthBetweenGroups      = %f\n", fLengthBetweenGroups);

            if ((fReflectGroupCircleRadius + fDirectGroupCircleRadius) >= fLengthBetweenGroups)
            {
              printf("OK = %d, %d\n", i, j);

              fss_cluster_data.LrfData[i].fDistance[0] = 10000;

              for (unsigned int k = 0; k < fss_cluster_data.LrfData[i].iScanID.size(); k++)
              {
                fss_cluster_data.LrfData[j].iScanID.push_back(fss_cluster_data.LrfData[i].iScanID[k]);
                fss_cluster_data.LrfData[j].bIsReflect.push_back(fss_cluster_data.LrfData[i].bIsReflect[k]);
                fss_cluster_data.LrfData[j].bIsForwardPoint.push_back(fss_cluster_data.LrfData[i].bIsForwardPoint[k]);
                fss_cluster_data.LrfData[j].fDistance.push_back(fss_cluster_data.LrfData[i].fDistance[k]);
                fss_cluster_data.LrfData[j].fIntensity.push_back(fss_cluster_data.LrfData[i].fIntensity[k]);
                fss_cluster_data.LrfData[j].fIntrinsicIntensity.push_back(
                    fss_cluster_data.LrfData[i].fIntrinsicIntensity[k]);
                fss_cluster_data.LrfData[j].fAcuteAngle.push_back(fss_cluster_data.LrfData[i].fAcuteAngle[k]);
                fss_cluster_data.LrfData[j].fX1.push_back(fss_cluster_data.LrfData[i].fX1[k]);
                fss_cluster_data.LrfData[j].fY1.push_back(fss_cluster_data.LrfData[i].fY1[k]);
                fss_cluster_data.LrfData[j].fX2.push_back(fss_cluster_data.LrfData[i].fX2[k]);
                fss_cluster_data.LrfData[j].fY2.push_back(fss_cluster_data.LrfData[i].fY2[k]);
              }

              //------------------------------------------------------
              fCenterX = fCenterY = 0.0;
              for (unsigned int l = 0; l < fss_cluster_data.LrfData[j].fX2.size(); l++)
              {
                fCenterX += fss_cluster_data.LrfData[j].fX2[l];
                fCenterY += fss_cluster_data.LrfData[j].fY2[l];
              }
              fCenterX = fCenterX / (float)fss_cluster_data.LrfData[j].fX2.size();
              fCenterY = fCenterY / (float)fss_cluster_data.LrfData[j].fY2.size();

              vstCenterX[j] = fCenterX;
              vstCenterY[j] = fCenterY;

              //------------------------------------------------------
              float fMax_size = 0.0;
              float fLength = 0.0;
              for (unsigned int l = 0; l < fss_cluster_data.LrfData[j].iScanID.size(); l++)
              {
                for (unsigned int m = l; m < fss_cluster_data.LrfData[j].iScanID.size(); m++)
                {
                  fLength = get_length(fss_cluster_data.LrfData[j].fX2[l], fss_cluster_data.LrfData[j].fY2[l],
                                       fss_cluster_data.LrfData[j].fX2[m], fss_cluster_data.LrfData[j].fY2[m]);

                  if (fLength >= fMax_size)
                  {
                    fMax_size = fLength;
                  }
                }
              }
              vstSize[j] = fMax_size;
            }
          }
        }
      }
    }
  }

  //--------------------------------------------------------------------------
  bool bIsSoloReflectGroup = true;
  int iSoloReflectGroupNum = 0;
  iLength = vstGroupID.size();
  for (unsigned int i = 0; i < iLength - iSoloReflectGroupNum;)
  {
    bIsSoloReflectGroup = true;

    for (unsigned int j = 0; j < fss_cluster_data.LrfData[i].iScanID.size(); j++)
    {
      if (fss_cluster_data.LrfData[i].bIsReflect[j] == false)  // include Direct Group
      {
        bIsSoloReflectGroup = false;
      }
    }

    if (fss_cluster_data.LrfData[i].fDistance[0] != 10000 && bUseSoloReflectCluster == true)
    {
      bIsSoloReflectGroup = false;
    }

    if (bIsSoloReflectGroup == true)
    {
      printf("group ID index = %d\n", i);
      vstGroupID.erase(vstGroupID.begin() + i);
      vstCenterX.erase(vstCenterX.begin() + i);
      vstCenterY.erase(vstCenterY.begin() + i);
      vstSize.erase(vstSize.begin() + i);
      vstAvgIntrinsicIntensity.erase(vstAvgIntrinsicIntensity.begin() + i);
      printf("group Data index = %d\n", i);
      fss_cluster_data.LrfData.erase(fss_cluster_data.LrfData.begin() + i);

      iSoloReflectGroupNum++;
    }
    else
    {
      i++;
    }
  }

  //--------------------------------------------------------------------------
  // occlusion
  //    bool bIsEmpty = fss_cluster_before.iID.empty();
  //    if(bIsEmpty==false)
  //    {
  //        for(unsigned int i=0 ; i<msg->fOcclusionX.size(); i++)
  //        {
  //            float fOX = msg->fOcclusionX[i];
  //            float fOY = msg->fOcclusionY[i];
  //
  //            for(unsigned int j=0 ; j<fss_cluster_before.iID.size(); j++)
  //            {
  //                float fCX = fss_cluster_before.fCenterX[j];
  //                float fCY = fss_cluster_before.fCenterY[j];
  //
  //                if( (fCX > (fOX-50)) && (fCX < (fOX+50)) && (fCY > (fOY-50)) && (fCY < (fOY+50)) )
  //                {
  //                    vstGroupID.push_back(iGroupID++);
  //                    vstCenterX.push_back(fss_cluster_before.fCenterX[j]);
  //                    vstCenterY.push_back(fss_cluster_before.fCenterY[j]);
  //                    vstAvgIntrinsicIntensity.push_back(fss_cluster_before.fAvgIntrinsicIntensity[j]);
  //                    fss_cluster_data.LrfData.push_back(fss_cluster_before.LrfData[j]);
  //                }
  //            }
  //        }
  //    }

  //--------------------------------------------------------------------------
  fss_cluster_data.header.frame_id = "fss_cluster_data";
  fss_cluster_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_cluster_data.tMeasuredTime = msg->tMeasuredTime;

  fss_cluster_data.iID = vstGroupID;
  fss_cluster_data.fCenterX = vstCenterX;
  fss_cluster_data.fCenterY = vstCenterY;
  fss_cluster_data.fSize = vstSize;
  fss_cluster_data.fAvgIntrinsicIntensity = vstAvgIntrinsicIntensity;

  fss_cluster_data.fOcclusionX = msg->fOcclusionX;
  fss_cluster_data.fOcclusionY = msg->fOcclusionY;

  //--------------------------------------------------------------------------
  rosPub.publish(fss_cluster_data);

  //--------------------------------------------------------------------------
  fss_cluster_before3.iID.clear();
  fss_cluster_before3.fCenterX.clear();
  fss_cluster_before3.fCenterY.clear();
  fss_cluster_before3.fAvgIntrinsicIntensity.clear();
  fss_cluster_before3.LrfData.clear();
  fss_cluster_before3.fOcclusionX.clear();
  fss_cluster_before3.fOcclusionY.clear();
  fss_cluster_before3 = fss_cluster_before2;

  fss_cluster_before2.iID.clear();
  fss_cluster_before2.fCenterX.clear();
  fss_cluster_before2.fCenterY.clear();
  fss_cluster_before2.fAvgIntrinsicIntensity.clear();
  fss_cluster_before2.LrfData.clear();
  fss_cluster_before2.fOcclusionX.clear();
  fss_cluster_before2.fOcclusionY.clear();
  fss_cluster_before2 = fss_cluster_before;

  fss_cluster_before.iID.clear();
  fss_cluster_before.fCenterX.clear();
  fss_cluster_before.fCenterY.clear();
  fss_cluster_before.fAvgIntrinsicIntensity.clear();
  fss_cluster_before.LrfData.clear();
  fss_cluster_before.fOcclusionX.clear();
  fss_cluster_before.fOcclusionY.clear();
  fss_cluster_before = fss_cluster_data;

  //--------------------------------------------------------------------------
  vstGroupID.clear();
  vstCenterX.clear();
  vstCenterY.clear();
  vstSize.clear();
  vstAvgIntrinsicIntensity.clear();

  vstScanID.clear();
  vstReflect.clear();
  vstIsForwardPoint.clear();
  vstDistance.clear();
  vstIntensity.clear();
  vstIntrinsicIntensity.clear();
  vstAcuteAngle.clear();
  vstX1.clear();
  vstY1.clear();
  vstX2.clear();
  vstY2.clear();

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed sending of fss_cluster_data #e");
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_cluster");
  ros::NodeHandle nh;
  rosSub = nh.subscribe("fss_tf_data", 10, lrfCallback);
  rosPub = nh.advertise< tms_msg_ss::fss_cluster_data >("fss_cluster_data", 10);

  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}
