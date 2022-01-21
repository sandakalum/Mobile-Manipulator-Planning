//------------------------------------------------------------------------------
// @file   : fss_track.cpp
// @brief  : object tracking
// @author : Yoonseok Pyo
// @version: Ver0.4.1 (since 2012.11.05)
// @date   : 2012.11.19
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <tms_ss_fss_exe/common.h>
#include <tms_ss_fss_exe/model.h>

#include <tms_msg_ss/fss_cluster_data.h>
#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_ss/fss_object_data.h>
#include <tms_msg_db/tmsdb_data.h>

//------------------------------------------------------------------------------
ros::Subscriber rosSub;
ros::Publisher rosPubSmartpal, rosPubRoomba, rosPubWagon, rosPubChair, rosPubUnknownClass;

vector< float > fRobotAngleArray;

float fWagonCenterXb2 = 0.0;
float fWagonCenterYb2 = 0.0;
float fWagonCenterXb1 = 0.0;
float fWagonCenterYb1 = 0.0;

vector< float > vfWagonBeforeVirtualPointX;
vector< float > vfWagonBeforeVirtualPointY;

float fChairCenterXb1 = 0.0;
float fChairCenterYb1 = 0.0;

vector< float > vfChairBeforeVirtualPointX;
vector< float > vfChairBeforeVirtualPointY;

POINT stSmartpalCenter;
POINT stRoombaCenter;
POINT stWagonCenter;
POINT stChairCenter;

vector< tms_msg_ss::fss_class_data > vstBefore_fss_unknown_cluster_data;

bool bDebugMode = false;
vector< float > x, y, t;

//------------------------------------------------------------------------------
float get_length(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//------------------------------------------------------------------------------
bool smartpalTracking(CLASS_DATA stClassData)
{
  ROS_INFO_STREAM("smartpalTracking #s");
  //--------------------------------------------------------------------------
  tms_msg_ss::fss_object_data fss_smartpal_data;

  vector< float > vfPointX;
  vector< float > vfPointY;
  vector< uint32_t > viIntensity;

  vector< BOUNDARY_POINT > vstBoundaryPonit;

  vector< float > vfSortedVirtualPointX;
  vector< float > vfSortedVirtualPointY;

  float fIntensityThreshold = 8000;  // Threshold of Intensity
  float fAcuteAngleThreshold = 65;   // Threshold of AcuteAngle  (degree)

  //--------------------------------------------------------------------------
  float fIntensityPointList[10] = {98.5, 87.3, 344.8, 195.2, 157.3, 141.4, 243.8, 223.0, 194.6, 325.0};

  POINT fPointList[11] = {{271, -159},
                          {245, -254},
                          {178, -310},
                          {-165, -275},
                          {-307, -141},
                          {-327, 15},
                          {-285, 150},
                          {-89, 295},
                          {134, 299},
                          {276, 166},
                          {0, 0}};

  //--------------------------------------------------------------------------
  if (stClassData.vstDatas.size() == 0)
    return false;

  //--------------------------------------------------------------------------
  // Save x,y distance and intensity data
  for (int i = 0; i < (int)(stClassData.vstDatas[0].fAcuteAngle.size()); i++)
  {
    if (stClassData.vstDatas[0].fAcuteAngle[i] < fAcuteAngleThreshold * DEG2RAD)
    {
      if (stClassData.vstDatas[0].bIsReflect[i] == false)
      {
        vfPointX.push_back(stClassData.vstDatas[0].fX2[i]);
        vfPointY.push_back(stClassData.vstDatas[0].fY2[i]);
        viIntensity.push_back(stClassData.vstDatas[0].fIntrinsicIntensity[i] > fIntensityThreshold ? 1 : 0);
      }
    }
  }

  //--------------------------------------------------------------------------
  printf("\nIntensityCode = ");
  for (int i = 0; i < (int)(viIntensity.size()); i++)
  {
    printf("[%d] ", viIntensity[i]);
  }

  //--------------------------------------------------------------------------
  BOUNDARY_POINT stBP;
  uint32_t iIntensityCode = 0;

  if (viIntensity.size() > 0)
    iIntensityCode = viIntensity[0];
  else
    return false;

  for (int i = 0; i < (int)(viIntensity.size()); i++)
  {
    if (viIntensity[i] != iIntensityCode && i != 0)
    {
      stBP.fPointX = vfPointX[i - 1];
      stBP.fPointY = vfPointY[i - 1];
      stBP.iIntensityCode = viIntensity[i - 1];
      vstBoundaryPonit.push_back(stBP);

      stBP.fPointX = vfPointX[i];
      stBP.fPointY = vfPointY[i];
      stBP.iIntensityCode = viIntensity[i];
      vstBoundaryPonit.push_back(stBP);

      iIntensityCode = viIntensity[i];
    }
  }

  //--------------------------------------------------------------------------
  uint32_t iBoundaryPointNum = vstBoundaryPonit.size();
  printf("\niBoundaryPointNum = %d\n", iBoundaryPointNum);

  bool bCheck = false;

  uint32_t iNumi = 0;
  uint32_t iNumj = 0;
  uint32_t iICode = 0;

  float fLength_max = 0.0;
  float fLength = 0.0;

  if (iBoundaryPointNum >= 2)
  {
    uint32_t i, j;

    for (i = 0; i < iBoundaryPointNum - 1; i++)
    {
      j = i + 1;

      if (vstBoundaryPonit[i].iIntensityCode == vstBoundaryPonit[j].iIntensityCode)
      {
        fLength = get_length(vstBoundaryPonit[i].fPointX, vstBoundaryPonit[i].fPointY, vstBoundaryPonit[j].fPointX,
                             vstBoundaryPonit[j].fPointY);

        if (fLength == 0)
          bCheck = false;
        else
          bCheck = true;

        iIntensityCode = vstBoundaryPonit[i].iIntensityCode;
        printf("length = %f, Code = %d\n", fLength, iIntensityCode);

        if (fLength > fLength_max)
        {
          iNumi = i;
          iNumj = j;
          fLength_max = fLength;
          iICode = iIntensityCode;
        }
      }
    }

    if (!bCheck)
      return false;

    printf("length_max = %f\n", fLength_max);
    printf("iICode = %d\n", iICode);
    printf("iNumi = %d\n", iNumi);
    printf("iNumj = %d\n", iNumj);

    //----------------------------------------------------------------------
    uint32_t iListNum, iListNum2, iListIndex, iListSize;
    float fLengthGap_min = 1000.0;
    float fLengthgap = 0.0;

    iListSize = (sizeof(fIntensityPointList) / sizeof(fIntensityPointList[0])) / 2;

    for (uint32_t k = 0; k < iListSize; k++)
    {
      if (iICode == 1)  // iICode = 1
        iListIndex = k * 2;
      else  // iICode = 0
        iListIndex = k * 2 + 1;

      fLengthgap = abs((float)(fLength_max - fIntensityPointList[iListIndex]));

      printf("iListNum = %d, fLengthgap = %f, iICode = %d\n", iListIndex, fLengthgap, iICode);

      if (fLengthgap < fLengthGap_min)
      {
        iListNum = iListIndex;
        fLengthGap_min = fLengthgap;
      }
    }

    printf("fLengthGap_min = %f\n", fLengthGap_min);
    printf("iICode = %d\n", iICode);
    printf("iListNum = %d\n", iListNum);

    //----------------------------------------------------------------------
    float fDifferenceDistanceX = vstBoundaryPonit[iNumi].fPointX - fPointList[iListNum].fX;
    float fDifferenceDistanceY = vstBoundaryPonit[iNumi].fPointY - fPointList[iListNum].fY;
    printf("fDifferenceDistanceX = %f\n", fDifferenceDistanceX);
    printf("fDifferenceDistanceY = %f\n", fDifferenceDistanceY);

    iListSize = sizeof(fPointList) / sizeof(fPointList[0]);
    printf("iListSize = %d\n", iListSize);

    for (uint32_t i = 0; i < iListSize; i++)
    {
      fPointList[i].fX = fPointList[i].fX + fDifferenceDistanceX;
      fPointList[i].fY = fPointList[i].fY + fDifferenceDistanceY;
    }

    //----------------------------------------------------------------------
    POINT fModelPA, fModelPB, fMeasuredPA, fMeasuredPB;

    if (iListNum >= iListSize - 2)
      iListNum2 = 0;
    else
      iListNum2 = iListNum + 1;

    fModelPA.fX = fPointList[iListNum].fX;
    fModelPA.fY = fPointList[iListNum].fY;
    fModelPB.fX = fPointList[iListNum2].fX;
    fModelPB.fY = fPointList[iListNum2].fY;
    fMeasuredPA.fX = vstBoundaryPonit[iNumi].fPointX;
    fMeasuredPA.fY = vstBoundaryPonit[iNumi].fPointY;
    fMeasuredPB.fX = vstBoundaryPonit[iNumj].fPointX;
    fMeasuredPB.fY = vstBoundaryPonit[iNumj].fPointY;

    printf("fModelPA    = %f, %f\n", fModelPA.fX, fModelPA.fY);
    printf("fModelPB    = %f, %f\n", fModelPB.fX, fModelPB.fY);
    printf("fMeasuredPA = %f, %f\n", fMeasuredPA.fX, fMeasuredPA.fY);
    printf("fMeasuredPB = %f, %f\n", fMeasuredPB.fX, fMeasuredPB.fY);
    printf("fCenter = %f, %f\n", fPointList[iListSize - 1].fX, fPointList[iListSize - 1].fY);

    float fModelPA_fMeasuredPB_D = get_length(fModelPA.fX, fModelPA.fY, fMeasuredPB.fX, fMeasuredPB.fY);
    float fModelPA_ModelPB_D = get_length(fModelPA.fX, fModelPA.fY, fModelPB.fX, fModelPB.fY);
    float fMeasuredPB_ModelPB_D = get_length(fMeasuredPB.fX, fMeasuredPB.fY, fModelPB.fX, fModelPB.fY);

    printf("fModelPA_fMeasuredPB_D = %f\n", fModelPA_fMeasuredPB_D);
    printf("fModelPA_ModelPB_D     = %f\n", fModelPA_ModelPB_D);
    printf("fMeasuredPB_ModelPB_D  = %f\n", fMeasuredPB_ModelPB_D);

    float fDifferenceAngle =
        acos((fModelPA_fMeasuredPB_D * fModelPA_fMeasuredPB_D + fModelPA_ModelPB_D * fModelPA_ModelPB_D -
              fMeasuredPB_ModelPB_D * fMeasuredPB_ModelPB_D) /
             (2 * fModelPA_fMeasuredPB_D * fModelPA_ModelPB_D));

    float dir = (fModelPB.fX - fModelPA.fX) * (fMeasuredPB.fY - fModelPA.fY) -
                (fMeasuredPB.fX - fModelPA.fX) * (fModelPB.fY - fModelPA.fY);

    if (dir == 0.0f)
      fDifferenceAngle = 0.0f;
    else if (dir < 0.0f)
      fDifferenceAngle = fDifferenceAngle * -1;
    else
      fDifferenceAngle = fDifferenceAngle * 1;

    printf("Theta = %f\n", fDifferenceAngle * RAD2DEG);

    //----------------------------------------------------------------------
    POINT stPoint;

    for (uint32_t i = 0; i < iListSize; i++)
    {
      stPoint.fX = fPointList[i].fX - fModelPA.fX;
      stPoint.fY = fPointList[i].fY - fModelPA.fY;

      fPointList[i].fX = (stPoint.fX) * cos(fDifferenceAngle) - (stPoint.fY) * sin(fDifferenceAngle);
      fPointList[i].fY = (stPoint.fX) * sin(fDifferenceAngle) + (stPoint.fY) * cos(fDifferenceAngle);

      fPointList[i].fX = fPointList[i].fX + fModelPA.fX;
      fPointList[i].fY = fPointList[i].fY + fModelPA.fY;
    }

    float fObjectCenterX = fPointList[iListSize - 1].fX;
    float fObjectCenterY = fPointList[iListSize - 1].fY;

    printf("fCenter = %f, %f\n", fObjectCenterX, fObjectCenterY);

    //----------------------------------------------------------------------
    // vstBoundaryPonit
    for (i = 0; i < iBoundaryPointNum; i++)
    {
      vfSortedVirtualPointX.push_back(vstBoundaryPonit[i].fPointX);
      vfSortedVirtualPointY.push_back(vstBoundaryPonit[i].fPointY);
    }

    //----------------------------------------------------------------------
    // Publish message
    fss_smartpal_data.header.frame_id = "fss_smartpal_data";
    fss_smartpal_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_smartpal_data.tMeasuredTime = stClassData.tMeasuredTime;

    tms_msg_db::tmsdb_data msgTmsdbData;

    msgTmsdbData.tMeasuredTime = stClassData.tMeasuredTime;

    msgTmsdbData.iType = TYPE_ROBOT;
    msgTmsdbData.iID = ID_SMARTPAL;

    msgTmsdbData.fX = fObjectCenterX;
    msgTmsdbData.fY = fObjectCenterY;
    msgTmsdbData.fZ = 0;
    msgTmsdbData.fTheta = fDifferenceAngle * RAD2DEG;

    msgTmsdbData.iPlace = ID_FLOOR;
    msgTmsdbData.iState = STATE_EXIST;

    fss_smartpal_data.msgTMSInfo.push_back(msgTmsdbData);

    fss_smartpal_data.iGroupsCount = 1;
    fss_smartpal_data.fClusterCenterX = stClassData.vfClusterCenterX;
    fss_smartpal_data.fClusterCenterY = stClassData.vfClusterCenterY;
    fss_smartpal_data.fClusterSize = stClassData.vfClusterSize;
    fss_smartpal_data.fAvgIntrinsicIntensity = stClassData.vfAvgIntrinsicIntensity;
    fss_smartpal_data.fVirtualPointX = vfSortedVirtualPointX;
    fss_smartpal_data.fVirtualPointY = vfSortedVirtualPointY;
    fss_smartpal_data.LrfData = stClassData.vstDatas;

    rosPubSmartpal.publish(fss_smartpal_data);

    stSmartpalCenter.fX = fss_smartpal_data.msgTMSInfo[0].fX;
    stSmartpalCenter.fY = fss_smartpal_data.msgTMSInfo[0].fY;

    //----------------------------------------------------------------------
    ROS_INFO_STREAM("Completed smartpalTracking #e");

    return true;
  }

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("Completed smartpalTracking #e");

  return false;
}

//------------------------------------------------------------------------------
bool roombaTracking(CLASS_DATA stClassData)
{
  ROS_INFO_STREAM("roombaTracking #s");
  //--------------------------------------------------------------------------
  tms_msg_ss::fss_object_data fss_roomba_data;

  vector< float > vfPointX;
  vector< float > vfPointY;
  vector< uint32_t > viIntensity;

  vector< BOUNDARY_POINT > vstBoundaryPonit;

  vector< float > vfSortedVirtualPointX;
  vector< float > vfSortedVirtualPointY;

  float fIntensityThreshold = 8000;  // Threshold of Intensity
  float fAcuteAngleThreshold = 65;   // Threshold of AcuteAngle  (degree)

  float fReflectIntensityThreshold = 2000;  // Threshold of Reflect Intensity

  //--------------------------------------------------------------------------
  float fIntensityPointList[8] = {53.6, 53.6, 199.8, 105.0, 154.7, 154.7, 105.0, 199.8};

  POINT fPointList[9] = {
      {162, 53}, {170, 0}, {162, -53}, {0, -170}, {-100, -138}, {-170, 0}, {-100, 138}, {0, 170}, {0, 0}};

  //--------------------------------------------------------------------------
  if (stClassData.vstDatas.size() == 0)
    return false;

  //--------------------------------------------------------------------------
  // Save x,y distance and intensity data
  for (int i = 0; i < (int)(stClassData.vstDatas[0].fAcuteAngle.size()); i++)
  {
    if (stClassData.vstDatas[0].bIsReflect[i] == false)
    {
      if (stClassData.vstDatas[0].fAcuteAngle[i] < fAcuteAngleThreshold * DEG2RAD)
      {
        vfPointX.push_back(stClassData.vstDatas[0].fX2[i]);
        vfPointY.push_back(stClassData.vstDatas[0].fY2[i]);
        viIntensity.push_back(stClassData.vstDatas[0].fIntrinsicIntensity[i] > fIntensityThreshold ? 1 : 0);
      }
    }
  }

  //--------------------------------------------------------------------------
  printf("\nIntensityCode = ");
  for (int i = 0; i < (int)(viIntensity.size()); i++)
  {
    printf("[%d] ", viIntensity[i]);
  }

  //--------------------------------------------------------------------------
  BOUNDARY_POINT stBP;
  uint32_t iIntensityCode = 0;

  if (viIntensity.size() > 0)
    iIntensityCode = viIntensity[0];
  else
    return false;

  for (int i = 0; i < (int)(viIntensity.size()); i++)
  {
    if (viIntensity[i] != iIntensityCode && i != 0)
    {
      stBP.fPointX = vfPointX[i - 1];
      stBP.fPointY = vfPointY[i - 1];
      stBP.iIntensityCode = viIntensity[i - 1];
      vstBoundaryPonit.push_back(stBP);

      stBP.fPointX = vfPointX[i];
      stBP.fPointY = vfPointY[i];
      stBP.iIntensityCode = viIntensity[i];
      vstBoundaryPonit.push_back(stBP);

      iIntensityCode = viIntensity[i];
    }
  }

  //--------------------------------------------------------------------------
  // Direct Beam
  uint32_t iBoundaryPointNum = vstBoundaryPonit.size();
  printf("\nDirect Beam iBoundaryPointNum = %d\n", iBoundaryPointNum);

  //--------------------------------------------------------------------------
  if (iBoundaryPointNum < 2)
  {
    //----------------------------------------------------------------------
    vfPointX.clear();
    vfPointY.clear();
    viIntensity.clear();
    vstBoundaryPonit.clear();

    //----------------------------------------------------------------------
    // Save x,y distance and intensity data
    for (int i = (int)(stClassData.vstDatas[0].fAcuteAngle.size() - 1); i > 0; i--)
    {
      if (stClassData.vstDatas[0].bIsReflect[i] == true)
      {
        if (stClassData.vstDatas[0].fAcuteAngle[i] < fAcuteAngleThreshold * DEG2RAD)
        {
          vfPointX.push_back(stClassData.vstDatas[0].fX2[i]);
          vfPointY.push_back(stClassData.vstDatas[0].fY2[i]);
          viIntensity.push_back(stClassData.vstDatas[0].fIntensity[i] > fReflectIntensityThreshold ? 1 : 0);
        }
      }
    }

    //---------------------------------------------------------------------
    printf("\nIntensityCode = ");
    for (int i = 0; i < (int)(viIntensity.size()); i++)
    {
      printf("[%d] ", viIntensity[i]);
    }

    //---------------------------------------------------------------------
    BOUNDARY_POINT stBP;
    uint32_t iIntensityCode = viIntensity[0];

    for (int i = 0; i < (int)(viIntensity.size()); i++)
    {
      if (viIntensity[i] != iIntensityCode && i != 0)
      {
        stBP.fPointX = vfPointX[i - 1];
        stBP.fPointY = vfPointY[i - 1];
        stBP.iIntensityCode = viIntensity[i - 1];
        vstBoundaryPonit.push_back(stBP);

        stBP.fPointX = vfPointX[i];
        stBP.fPointY = vfPointY[i];
        stBP.iIntensityCode = viIntensity[i];
        vstBoundaryPonit.push_back(stBP);

        iIntensityCode = viIntensity[i];
      }
    }
    // Reflect Beam
    iBoundaryPointNum = vstBoundaryPonit.size();
    printf("\nReflect Beam iBoundaryPointNum = %d\n", iBoundaryPointNum);
  }

  //--------------------------------------------------------------------------
  bool bCheck = false;

  uint32_t iNumi = 0;
  uint32_t iNumj = 0;
  uint32_t iICode = 0;

  float fLength_max = 0.0;
  float fLength = 0.0;

  if (iBoundaryPointNum >= 2)
  {
    uint32_t i, j;

    for (i = 0; i < iBoundaryPointNum - 1; i++)
    {
      j = i + 1;

      if (vstBoundaryPonit[i].iIntensityCode == vstBoundaryPonit[j].iIntensityCode)
      {
        fLength = get_length(vstBoundaryPonit[i].fPointX, vstBoundaryPonit[i].fPointY, vstBoundaryPonit[j].fPointX,
                             vstBoundaryPonit[j].fPointY);

        if (fLength == 0)
          bCheck = false;
        else
          bCheck = true;

        iIntensityCode = vstBoundaryPonit[i].iIntensityCode;
        printf("length = %f, Code = %d\n", fLength, iIntensityCode);

        if (fLength > fLength_max)
        {
          iNumi = i;
          iNumj = j;
          fLength_max = fLength;
          iICode = iIntensityCode;
        }
      }
    }

    if (!bCheck)
      return false;

    printf("length_max = %f\n", fLength_max);
    printf("iICode = %d\n", iICode);
    printf("iNumi = %d\n", iNumi);
    printf("iNumj = %d\n", iNumj);

    //----------------------------------------------------------------------
    uint32_t iListNum, iListNum2, iListIndex, iListSize;
    float fLengthGap_min = 1000.0;
    float fLengthgap = 0.0;

    iListSize = (sizeof(fIntensityPointList) / sizeof(fIntensityPointList[0])) / 2;

    for (uint32_t k = 0; k < iListSize; k++)
    {
      if (iICode == 1)  // iICode = 1
        iListIndex = k * 2;
      else  // iICode = 0
        iListIndex = k * 2 + 1;

      fLengthgap = abs((float)(fLength_max - fIntensityPointList[iListIndex]));

      printf("iListNum = %d, fLengthgap = %f, iICode = %d\n", iListIndex, fLengthgap, iICode);

      if (fLengthgap < fLengthGap_min)
      {
        iListNum = iListIndex;
        fLengthGap_min = fLengthgap;
      }
    }

    printf("fLengthGap_min = %f\n", fLengthGap_min);
    printf("iICode = %d\n", iICode);
    printf("iListNum = %d\n", iListNum);

    //----------------------------------------------------------------------
    float fDifferenceDistanceX = vstBoundaryPonit[iNumi].fPointX - fPointList[iListNum].fX;
    float fDifferenceDistanceY = vstBoundaryPonit[iNumi].fPointY - fPointList[iListNum].fY;
    printf("fDifferenceDistanceX = %f\n", fDifferenceDistanceX);
    printf("fDifferenceDistanceY = %f\n", fDifferenceDistanceY);

    iListSize = sizeof(fPointList) / sizeof(fPointList[0]);
    printf("iListSize = %d\n", iListSize);

    for (uint32_t i = 0; i < iListSize; i++)
    {
      fPointList[i].fX = fPointList[i].fX + fDifferenceDistanceX;
      fPointList[i].fY = fPointList[i].fY + fDifferenceDistanceY;
    }

    //----------------------------------------------------------------------
    POINT fModelPA, fModelPB, fMeasuredPA, fMeasuredPB;

    if (iListNum >= iListSize - 2)
      iListNum2 = 0;
    else
      iListNum2 = iListNum + 1;

    fModelPA.fX = fPointList[iListNum].fX;
    fModelPA.fY = fPointList[iListNum].fY;
    fModelPB.fX = fPointList[iListNum2].fX;
    fModelPB.fY = fPointList[iListNum2].fY;
    fMeasuredPA.fX = vstBoundaryPonit[iNumi].fPointX;
    fMeasuredPA.fY = vstBoundaryPonit[iNumi].fPointY;
    fMeasuredPB.fX = vstBoundaryPonit[iNumj].fPointX;
    fMeasuredPB.fY = vstBoundaryPonit[iNumj].fPointY;

    printf("fModelPA    = %f, %f\n", fModelPA.fX, fModelPA.fY);
    printf("fModelPB    = %f, %f\n", fModelPB.fX, fModelPB.fY);
    printf("fMeasuredPA = %f, %f\n", fMeasuredPA.fX, fMeasuredPA.fY);
    printf("fMeasuredPB = %f, %f\n", fMeasuredPB.fX, fMeasuredPB.fY);
    printf("fCenter = %f, %f\n", fPointList[iListSize - 1].fX, fPointList[iListSize - 1].fY);

    float fModelPA_fMeasuredPB_D = get_length(fModelPA.fX, fModelPA.fY, fMeasuredPB.fX, fMeasuredPB.fY);
    float fModelPA_ModelPB_D = get_length(fModelPA.fX, fModelPA.fY, fModelPB.fX, fModelPB.fY);
    float fMeasuredPB_ModelPB_D = get_length(fMeasuredPB.fX, fMeasuredPB.fY, fModelPB.fX, fModelPB.fY);

    printf("fModelPA_fMeasuredPB_D = %f\n", fModelPA_fMeasuredPB_D);
    printf("fModelPA_ModelPB_D     = %f\n", fModelPA_ModelPB_D);
    printf("fMeasuredPB_ModelPB_D  = %f\n", fMeasuredPB_ModelPB_D);

    float fDifferenceAngle =
        acos((fModelPA_fMeasuredPB_D * fModelPA_fMeasuredPB_D + fModelPA_ModelPB_D * fModelPA_ModelPB_D -
              fMeasuredPB_ModelPB_D * fMeasuredPB_ModelPB_D) /
             (2 * fModelPA_fMeasuredPB_D * fModelPA_ModelPB_D));

    float dir = (fModelPB.fX - fModelPA.fX) * (fMeasuredPB.fY - fModelPA.fY) -
                (fMeasuredPB.fX - fModelPA.fX) * (fModelPB.fY - fModelPA.fY);

    if (dir == 0.0f)
      fDifferenceAngle = 0.0f;
    else if (dir < 0.0f)
      fDifferenceAngle = fDifferenceAngle * -1;
    else
      fDifferenceAngle = fDifferenceAngle * 1;

    printf("Theta = %f\n", fDifferenceAngle * RAD2DEG);

    //----------------------------------------------------------------------
    POINT stPoint;

    for (uint32_t i = 0; i < iListSize; i++)
    {
      stPoint.fX = fPointList[i].fX - fModelPA.fX;
      stPoint.fY = fPointList[i].fY - fModelPA.fY;

      fPointList[i].fX = (stPoint.fX) * cos(fDifferenceAngle) - (stPoint.fY) * sin(fDifferenceAngle);
      fPointList[i].fY = (stPoint.fX) * sin(fDifferenceAngle) + (stPoint.fY) * cos(fDifferenceAngle);

      fPointList[i].fX = fPointList[i].fX + fModelPA.fX;
      fPointList[i].fY = fPointList[i].fY + fModelPA.fY;
    }

    float fObjectCenterX = fPointList[iListSize - 1].fX;
    float fObjectCenterY = fPointList[iListSize - 1].fY;

    printf("fCenter = %f, %f\n", fObjectCenterX, fObjectCenterY);

    //----------------------------------------------------------------------
    // vstBoundaryPonit
    for (i = 0; i < iBoundaryPointNum; i++)
    {
      vfSortedVirtualPointX.push_back(vstBoundaryPonit[i].fPointX);
      vfSortedVirtualPointY.push_back(vstBoundaryPonit[i].fPointY);
    }

    //----------------------------------------------------------------------
    // Publish message
    fRobotAngleArray.push_back(fDifferenceAngle * RAD2DEG);

    // printf("x size = %d\n",x.size());

    if (fRobotAngleArray.size() == 9)
    {
      sort(fRobotAngleArray.begin(), fRobotAngleArray.end());

      fss_roomba_data.header.frame_id = "fss_roomba_data";
      fss_roomba_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
      fss_roomba_data.tMeasuredTime = stClassData.tMeasuredTime;

      tms_msg_db::tmsdb_data msgTmsdbData;

      msgTmsdbData.tMeasuredTime = stClassData.tMeasuredTime;

      msgTmsdbData.iType = TYPE_ROBOT;
      msgTmsdbData.iID = ID_ROOMBA;

      msgTmsdbData.fX = fObjectCenterX;
      msgTmsdbData.fY = fObjectCenterY;
      msgTmsdbData.fZ = 0;
      msgTmsdbData.fTheta = fRobotAngleArray[4];

      /*
      x.push_back(fObjectCenterX);
      y.push_back(fObjectCenterY);
      t.push_back(fRobotAngleArray[4]);
      if(x.size() == 100)
      {
          for(uint32_t xi=0; xi < x.size(); xi++)
          {
              printf("%f, %f, %f\n",x[xi],y[xi],t[xi]);
          }

          x.clear();
          y.clear();
          t.clear();
      }
      */
      msgTmsdbData.iPlace = ID_FLOOR;
      msgTmsdbData.iState = STATE_EXIST;

      fss_roomba_data.msgTMSInfo.push_back(msgTmsdbData);

      fss_roomba_data.iGroupsCount = 1;
      fss_roomba_data.fClusterCenterX = stClassData.vfClusterCenterX;
      fss_roomba_data.fClusterCenterY = stClassData.vfClusterCenterY;
      fss_roomba_data.fClusterSize = stClassData.vfClusterSize;
      fss_roomba_data.fAvgIntrinsicIntensity = stClassData.vfAvgIntrinsicIntensity;
      fss_roomba_data.fVirtualPointX = vfSortedVirtualPointX;
      fss_roomba_data.fVirtualPointY = vfSortedVirtualPointY;
      fss_roomba_data.LrfData = stClassData.vstDatas;

      rosPubRoomba.publish(fss_roomba_data);

      fRobotAngleArray.clear();

      stRoombaCenter.fX = fss_roomba_data.msgTMSInfo[0].fX;
      stRoombaCenter.fY = fss_roomba_data.msgTMSInfo[0].fY;

      //------------------------------------------------------------------
      ROS_INFO_STREAM("completed roombaTracking #e");

      return true;
    }
    return true;
  }

  printf("matching failed : iBoundaryPointNum <= 2\n");

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed roombaTracking #e");

  return false;
}

//------------------------------------------------------------------------------
bool wagonTracking(CLASS_DATA stClassData)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("wagonTracking #s");

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_object_data fss_wagon_data;

  vector< float > vfClusterCenterX;
  vector< float > vfClusterCenterY;
  vector< float > vfClusterSize;
  vector< float > vfAvgIntrinsicIntensity;
  vector< tms_msg_ss::fss_tf_datas > vstDatas;

  vector< float > vfVirtualPointX;
  vector< float > vfVirtualPointY;
  vector< float > vfSortedVirtualPointX;
  vector< float > vfSortedVirtualPointY;

  float fDirection = 0.0;
  float fObjectCenterX = 0.0;
  float fObjectCenterY = 0.0;

  uint32_t iGroupsCount = 0;

  unsigned int iNumi = 0;
  unsigned int iNumj = 0;

  // float fMinDiagonalLength= 580; // Diagonal length of wagon (Min)
  // float fMaxFrontLength   = 300; // Front length of wagon (Max)
  // float fMinFrontLength   = 230; // Front length of wagon (Min)
  // float fMaxSideLength    = 580; // Side  length of wagon (Max)
  // float fMinSideLength    = 500; // Side  length of wagon (Min)

  float fMinDiagonalLength = 435;  // Diagonal length of wagon (Min)
  float fMaxFrontLength = 300;     // Front length of wagon (Max)
  float fMinFrontLength = 230;     // Front length of wagon (Min)
  float fMaxSideLength = 455;      // Side  length of wagon (Max)
  float fMinSideLength = 375;      // Side  length of wagon (Min)

  //--------------------------------------------------------------------------
  for (unsigned int i = 0; i < stClassData.vfClusterCenterX.size(); i++)
  {
    float fDistance = 0.0;
    float fMinDistance = 10000.0;

    for (unsigned int j = 0; j < stClassData.vfClusterCenterX.size(); j++)
    {
      if (i == j)
        continue;

      fDistance = get_length(stClassData.vfClusterCenterX[i], stClassData.vfClusterCenterY[i],
                             stClassData.vfClusterCenterX[j], stClassData.vfClusterCenterY[j]);

      if (fDistance <= fMinDistance)
      {
        fMinDistance = fDistance;
      }
    }

    if (fMinDistance < 1000)  //  < 1000mm
    {
      iGroupsCount++;

      vfClusterCenterX.push_back(stClassData.vfClusterCenterX[i]);
      vfClusterCenterY.push_back(stClassData.vfClusterCenterY[i]);
      vfClusterSize.push_back(stClassData.vfClusterSize[i]);
      vfAvgIntrinsicIntensity.push_back(stClassData.vfAvgIntrinsicIntensity[i]);
      vstDatas.push_back(stClassData.vstDatas[i]);
    }
  }

  //--------------------------------------------------------------------------
  // Calculate max length of wheels
  unsigned int iWheelNum = vfClusterCenterX.size();
  printf("iWheelNum = %d\n", iWheelNum);

  float fLength_max = 0.0;
  float fLength = 0.0;

  if (iWheelNum > 0)
  {
    for (unsigned int i = 0; i < iWheelNum - 1; i++)
    {
      for (unsigned int j = i + 1; j < iWheelNum; j++)
      {
        fLength = get_length(vfClusterCenterX[i], vfClusterCenterY[i], vfClusterCenterX[j], vfClusterCenterY[j]);

        printf("length = %f\n", fLength);

        if (fLength > fLength_max)
        {
          iNumi = i;
          iNumj = j;
          fLength_max = fLength;
        }
      }
    }
  }

  printf("length_max = %f\n", fLength_max);
  printf("iNumi = %d\n", iNumi);
  printf("iNumj = %d\n", iNumj);

  //--------------------------------------------------------------------------
  // Calculate center of object
  if (iWheelNum > 2)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      fObjectCenterX = (vfClusterCenterX[iNumi] + vfClusterCenterX[iNumj]) / 2;
      fObjectCenterY = (vfClusterCenterY[iNumi] + vfClusterCenterY[iNumj]) / 2;
    }
    else
    {
      fObjectCenterX = 0.0;
      fObjectCenterY = 0.0;
    }
  }
  else if (iWheelNum == 2)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      fObjectCenterX = (vfClusterCenterX[iNumi] + vfClusterCenterX[iNumj]) / 2;
      fObjectCenterY = (vfClusterCenterY[iNumi] + vfClusterCenterY[iNumj]) / 2;
    }
    else if (fLength_max > fMinSideLength && fLength_max < fMaxSideLength)
    {
      float fTheta = acos(abs(vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) / fLength_max);

      fTheta =
          (vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) * (vfClusterCenterY[iNumi] - vfClusterCenterY[iNumj]) >
                  0 ?
              fTheta :
              M_PI - fTheta;

      float fObjectCenterX1 = (270.0) * cos(fTheta) - (140.0) * sin(fTheta);
      float fObjectCenterY1 = (270.0) * sin(fTheta) + (140.0) * cos(fTheta);

      float fObjectCenterX2 = (270.0) * cos(fTheta) - (-140.0) * sin(fTheta);
      float fObjectCenterY2 = (270.0) * sin(fTheta) + (-140.0) * cos(fTheta);

      int iNumBase = vfClusterCenterY[iNumi] < vfClusterCenterY[iNumj] ? iNumi : iNumj;

      fObjectCenterX1 = fObjectCenterX1 + vfClusterCenterX[iNumBase];
      fObjectCenterY1 = fObjectCenterY1 + vfClusterCenterY[iNumBase];

      fObjectCenterX2 = fObjectCenterX2 + vfClusterCenterX[iNumBase];
      fObjectCenterY2 = fObjectCenterY2 + vfClusterCenterY[iNumBase];

      float fLength1 = get_length(fObjectCenterX1, fObjectCenterY1, fWagonCenterXb1, fWagonCenterYb1);
      float fLength2 = get_length(fObjectCenterX2, fObjectCenterY2, fWagonCenterXb1, fWagonCenterYb1);

      if (fLength1 < fLength2)
      {
        fObjectCenterX = fObjectCenterX1;
        fObjectCenterY = fObjectCenterY1;
      }
      else
      {
        fObjectCenterX = fObjectCenterX2;
        fObjectCenterY = fObjectCenterY2;
      }
    }
    else if (fLength_max > fMinFrontLength && fLength_max < fMaxFrontLength)
    {
      float fTheta = acos(abs(vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) / fLength_max);

      fTheta =
          (vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) * (vfClusterCenterY[iNumi] - vfClusterCenterY[iNumj]) >
                  0 ?
              fTheta :
              M_PI - fTheta;

      float fObjectCenterX1 = (140.0) * cos(fTheta) - (270.0) * sin(fTheta);
      float fObjectCenterY1 = (140.0) * sin(fTheta) + (270.0) * cos(fTheta);

      float fObjectCenterX2 = (140.0) * cos(fTheta) - (-270.0) * sin(fTheta);
      float fObjectCenterY2 = (140.0) * sin(fTheta) + (-270.0) * cos(fTheta);

      int iNumBase = vfClusterCenterY[iNumi] < vfClusterCenterY[iNumj] ? iNumi : iNumj;

      fObjectCenterX1 = fObjectCenterX1 + vfClusterCenterX[iNumBase];
      fObjectCenterY1 = fObjectCenterY1 + vfClusterCenterY[iNumBase];

      fObjectCenterX2 = fObjectCenterX2 + vfClusterCenterX[iNumBase];
      fObjectCenterY2 = fObjectCenterY2 + vfClusterCenterY[iNumBase];

      float fLength1 = get_length(fObjectCenterX1, fObjectCenterY1, fWagonCenterXb1, fWagonCenterYb1);
      float fLength2 = get_length(fObjectCenterX2, fObjectCenterY2, fWagonCenterXb1, fWagonCenterYb1);

      if (fLength1 < fLength2)
      {
        fObjectCenterX = fObjectCenterX1;
        fObjectCenterY = fObjectCenterY1;
      }
      else
      {
        fObjectCenterX = fObjectCenterX2;
        fObjectCenterY = fObjectCenterY2;
      }
    }
    else
    {
      fObjectCenterX = 0.0;
      fObjectCenterY = 0.0;

      return false;
    }
  }
  else
  {
    fObjectCenterX = 0.0;
    fObjectCenterY = 0.0;

    return false;
  }

  //--------------------------------------------------------------------------
  // Calculate direction of object
  if (fObjectCenterX != 0 && fObjectCenterY != 0 && fWagonCenterXb1 != 0 && fWagonCenterYb1 != 0)
  {
    float fD = get_length(fObjectCenterX, fObjectCenterY, fWagonCenterXb1, fWagonCenterYb1);
    float fDx = abs(fObjectCenterX - fWagonCenterXb1);

    if (fD > 20)  // more than 20mm
    {
      if (fObjectCenterX - fWagonCenterXb1 > 0 && fObjectCenterY - fWagonCenterYb1 > 0)
        fDirection = acos(fDx / fD) * RAD2DEG;
      else if (fObjectCenterX - fWagonCenterXb1 < 0 && fObjectCenterY - fWagonCenterYb1 > 0)
        fDirection = 180 - (acos(fDx / fD) * RAD2DEG);
      else if (fObjectCenterX - fWagonCenterXb1 < 0 && fObjectCenterY - fWagonCenterYb1 < 0)
        fDirection = 180 + (acos(fDx / fD) * RAD2DEG);
      else if (fObjectCenterX - fWagonCenterXb1 > 0 && fObjectCenterY - fWagonCenterYb1 < 0)
        fDirection = 360 - (acos(fDx / fD) * RAD2DEG);
    }
  }
  else
  {
    fDirection = 0.0;
  }

  printf("fDirection = %f\n", fDirection);

  fWagonCenterXb1 = fObjectCenterX;
  fWagonCenterYb1 = fObjectCenterY;

  //--------------------------------------------------------------------------
  // Estimate four-point of rectangle(Wagon)
  float fDx = 0.0;
  float fDy = 0.0;

  vfVirtualPointX.clear();
  vfVirtualPointY.clear();

  if (iWheelNum == 4 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    for (int i = 0; i < 4; i++)
    {
      vfVirtualPointX.push_back(vfClusterCenterX[i]);
      vfVirtualPointY.push_back(vfClusterCenterY[i]);
    }
  }
  else if (iWheelNum == 3 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    for (unsigned int i = 0; i < 3; i++)
    {
      if (i != iNumi && i != iNumj)
      {
        fDx = (fObjectCenterX > vfClusterCenterX[i] ? fObjectCenterX - vfClusterCenterX[i] :
                                                      vfClusterCenterX[i] - fObjectCenterX);
        fDy = (fObjectCenterY > vfClusterCenterY[i] ? fObjectCenterY - vfClusterCenterY[i] :
                                                      vfClusterCenterY[i] - fObjectCenterY);

        if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * -1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * 1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * 1;
          fDy = fDy * 1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * -1;
          fDy = fDy * 1;
        }

        vfVirtualPointX.push_back(fObjectCenterX + fDx);
        vfVirtualPointY.push_back(fObjectCenterY + fDy);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
        printf("fDx = %f\n", fDx);
        printf("fDy = %f\n", fDy);
        printf("i = %d\n", i);
      }
      else
      {
        printf("i = %d\n", i);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
      }
    }
  }
  else if (iWheelNum == 2 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      vfVirtualPointX = vfWagonBeforeVirtualPointX;
      vfVirtualPointY = vfWagonBeforeVirtualPointY;
    }
    else if (fLength_max > fMinFrontLength && fLength_max < fMaxSideLength)
    {
      for (unsigned int i = 0; i < 2; i++)
      {
        fDx = (fObjectCenterX > vfClusterCenterX[i] ? fObjectCenterX - vfClusterCenterX[i] :
                                                      vfClusterCenterX[i] - fObjectCenterX);
        fDy = (fObjectCenterY > vfClusterCenterY[i] ? fObjectCenterY - vfClusterCenterY[i] :
                                                      vfClusterCenterY[i] - fObjectCenterY);

        if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * -1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * 1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * 1;
          fDy = fDy * 1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * -1;
          fDy = fDy * 1;
        }

        vfVirtualPointX.push_back(fObjectCenterX + fDx);
        vfVirtualPointY.push_back(fObjectCenterY + fDy);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
        printf("fDx = %f\n", fDx);
        printf("fDy = %f\n", fDy);
        printf("i = %d\n", i);
      }
    }
  }
  else
  {
    if (vfWagonBeforeVirtualPointX.size() == 4 && vfWagonBeforeVirtualPointY.size() == 4)
    {
      vfVirtualPointX = vfWagonBeforeVirtualPointX;
      vfVirtualPointY = vfWagonBeforeVirtualPointY;
    }
    else
    {
      for (unsigned int i = 0; i < 4; i++)
      {
        vfVirtualPointX.push_back(0.0);
        vfVirtualPointY.push_back(0.0);
      }
    }
  }

  vfWagonBeforeVirtualPointX.clear();
  vfWagonBeforeVirtualPointY.clear();
  vfWagonBeforeVirtualPointX = vfVirtualPointX;
  vfWagonBeforeVirtualPointY = vfVirtualPointY;

  //--------------------------------------------------------------------------
  // Sort virtual point
  if (vfVirtualPointX.size() == 4 && vfVirtualPointY.size() == 4)
  {
    //----------------------------------------------------------------------
    float fMaximum = 0.0;
    float fMinimum = 6000.0;
    float fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointY[i];

      if (fPosition > fMaximum)
      {
        iNumi = i;
        fMaximum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMinimum = 6000.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointX[i];

      if (fPosition < fMinimum)
      {
        iNumi = i;
        fMinimum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMinimum = 4000.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointY[i];

      if (fPosition < fMinimum)
      {
        iNumi = i;
        fMinimum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMaximum = 0.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointX[i];

      if (fPosition > fMaximum)
      {
        iNumi = i;
        fMaximum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
  }
  else
  {
    vfSortedVirtualPointX.clear();
    vfSortedVirtualPointY.clear();
    for (unsigned int i = 0; i < 4; i++)
    {
      vfSortedVirtualPointX.push_back(0.0);
      vfSortedVirtualPointY.push_back(0.0);
    }
  }

  //--------------------------------------------------------------------------
  // Publish message
  fss_wagon_data.header.frame_id = "fss_wagon_data";
  fss_wagon_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_wagon_data.tMeasuredTime = stClassData.tMeasuredTime;

  tms_msg_db::tmsdb_data msgTmsdbData;

  msgTmsdbData.tMeasuredTime = stClassData.tMeasuredTime;

  msgTmsdbData.iType = TYPE_MOVABLE_FUNITURE;
  msgTmsdbData.iID = ID_WAGON1;

  msgTmsdbData.fX = fObjectCenterX;
  msgTmsdbData.fY = fObjectCenterY;
  msgTmsdbData.fZ = 0;
  msgTmsdbData.fTheta = fDirection * DEG2RAD;

  msgTmsdbData.iPlace = ID_FLOOR;
  msgTmsdbData.iState = STATE_EXIST;

  fss_wagon_data.msgTMSInfo.push_back(msgTmsdbData);

  fss_wagon_data.iGroupsCount = iGroupsCount;
  fss_wagon_data.fClusterCenterX = vfClusterCenterX;
  fss_wagon_data.fClusterCenterY = vfClusterCenterY;
  fss_wagon_data.fClusterSize = vfClusterSize;
  fss_wagon_data.fAvgIntrinsicIntensity = vfAvgIntrinsicIntensity;
  fss_wagon_data.fVirtualPointX = vfSortedVirtualPointX;
  fss_wagon_data.fVirtualPointY = vfSortedVirtualPointY;
  fss_wagon_data.LrfData = vstDatas;

  rosPubWagon.publish(fss_wagon_data);

  stWagonCenter.fX = fss_wagon_data.msgTMSInfo[0].fX;
  stWagonCenter.fY = fss_wagon_data.msgTMSInfo[0].fY;

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed wagonTracking #e");

  return true;
}

//------------------------------------------------------------------------------
bool chairTracking(CLASS_DATA stClassData)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("chairTracking #s");

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_object_data fss_chair_data;

  vector< float > vfClusterCenterX;
  vector< float > vfClusterCenterY;
  vector< float > vfClusterSize;
  vector< float > vfAvgIntrinsicIntensity;
  vector< tms_msg_ss::fss_tf_datas > vstDatas;

  vector< float > vfVirtualPointX;
  vector< float > vfVirtualPointY;
  vector< float > vfSortedVirtualPointX;
  vector< float > vfSortedVirtualPointY;

  uint32_t iGroupsCount = 0;

  float fObjectCenterX = 0.0;
  float fObjectCenterY = 0.0;

  unsigned int iNumi = 0;
  unsigned int iNumj = 0;

  // float fMaxDiagonalLength = 565; // Diagonal length of chair (Max)
  float fMinDiagonalLength = 520;  // Diagonal length of chair (Min)
  float fMaxFrontLength = 355;     // Front length of chair (Max)
  float fMinFrontLength = 325;     // Front length of chair (Min)
  float fMaxSideLength = 420;      // Side  length of chair (Max)
  float fMinSideLength = 400;      // Side  length of chair (Min)

  //--------------------------------------------------------------------------
  for (unsigned int i = 0; i < stClassData.vfClusterCenterX.size(); i++)
  {
    float fDistance = 0.0;
    float fMinDistance = 10000.0;

    for (unsigned int j = 0; j < stClassData.vfClusterCenterX.size(); j++)
    {
      if (i == j)
        continue;

      fDistance = get_length(stClassData.vfClusterCenterX[i], stClassData.vfClusterCenterY[i],
                             stClassData.vfClusterCenterX[j], stClassData.vfClusterCenterY[j]);

      if (fDistance <= fMinDistance)
      {
        fMinDistance = fDistance;
      }
    }

    if (fMinDistance < 550)  //  < 500mm
    {
      iGroupsCount++;

      vfClusterCenterX.push_back(stClassData.vfClusterCenterX[i]);
      vfClusterCenterY.push_back(stClassData.vfClusterCenterY[i]);
      vfClusterSize.push_back(stClassData.vfClusterSize[i]);
      vfAvgIntrinsicIntensity.push_back(stClassData.vfAvgIntrinsicIntensity[i]);
      vstDatas.push_back(stClassData.vstDatas[i]);
    }
  }
  //--------------------------------------------------------------------------
  // Calculate max length of chair leg
  unsigned int iChairLeglNum = vfClusterCenterX.size();

  if (iChairLeglNum == 0)
    return false;

  printf("iChairLeglNum = %d\n", iChairLeglNum);

  float fLength_max = 0.0;
  float fLength = 0.0;

  if (iChairLeglNum > 0)
  {
    for (unsigned int i = 0; i < iChairLeglNum - 1; i++)
    {
      for (unsigned int j = i + 1; j < iChairLeglNum; j++)
      {
        fLength = get_length(vfClusterCenterX[i], vfClusterCenterY[i], vfClusterCenterX[j], vfClusterCenterY[j]);

        printf("length = %f\n", fLength);

        if (fLength > fLength_max)
        {
          iNumi = i;
          iNumj = j;
          fLength_max = fLength;
        }
      }
    }
  }

  printf("length_max = %f\n", fLength_max);
  printf("iNumi = %d\n", iNumi);
  printf("iNumj = %d\n", iNumj);

  //--------------------------------------------------------------------------
  // Calculate center of object
  if (iChairLeglNum > 2)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      fObjectCenterX = (vfClusterCenterX[iNumi] + vfClusterCenterX[iNumj]) / 2;
      fObjectCenterY = (vfClusterCenterY[iNumi] + vfClusterCenterY[iNumj]) / 2;
    }
    else
    {
      fObjectCenterX = 0.0;
      fObjectCenterY = 0.0;

      return false;
    }
  }
  else if (iChairLeglNum == 2)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      fObjectCenterX = (vfClusterCenterX[iNumi] + vfClusterCenterX[iNumj]) / 2;
      fObjectCenterY = (vfClusterCenterY[iNumi] + vfClusterCenterY[iNumj]) / 2;
    }
    else if (fLength_max > fMinSideLength && fLength_max < fMaxSideLength)
    {
      float fTheta = acos(abs(vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) / fLength_max);

      fTheta =
          (vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) * (vfClusterCenterY[iNumi] - vfClusterCenterY[iNumj]) >
                  0 ?
              fTheta :
              M_PI - fTheta;

      float fObjectCenterX1 = (203.0) * cos(fTheta) - (170.0) * sin(fTheta);
      float fObjectCenterY1 = (203.0) * sin(fTheta) + (170.0) * cos(fTheta);

      float fObjectCenterX2 = (203.0) * cos(fTheta) - (-170.0) * sin(fTheta);
      float fObjectCenterY2 = (203.0) * sin(fTheta) + (-170.0) * cos(fTheta);

      int iNumBase = vfClusterCenterY[iNumi] < vfClusterCenterY[iNumj] ? iNumi : iNumj;

      fObjectCenterX1 = fObjectCenterX1 + vfClusterCenterX[iNumBase];
      fObjectCenterY1 = fObjectCenterY1 + vfClusterCenterY[iNumBase];

      fObjectCenterX2 = fObjectCenterX2 + vfClusterCenterX[iNumBase];
      fObjectCenterY2 = fObjectCenterY2 + vfClusterCenterY[iNumBase];

      float fLength1 = get_length(fObjectCenterX1, fObjectCenterY1, fChairCenterXb1, fChairCenterYb1);
      float fLength2 = get_length(fObjectCenterX2, fObjectCenterY2, fChairCenterXb1, fChairCenterYb1);

      if (fLength1 < fLength2)
      {
        fObjectCenterX = fObjectCenterX1;
        fObjectCenterY = fObjectCenterY1;
      }
      else
      {
        fObjectCenterX = fObjectCenterX2;
        fObjectCenterY = fObjectCenterY2;
      }
    }
    else if (fLength_max > fMinFrontLength && fLength_max < fMaxFrontLength)
    {
      float fTheta = acos(abs(vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) / fLength_max);

      fTheta =
          (vfClusterCenterX[iNumi] - vfClusterCenterX[iNumj]) * (vfClusterCenterY[iNumi] - vfClusterCenterY[iNumj]) >
                  0 ?
              fTheta :
              M_PI - fTheta;

      float fObjectCenterX1 = (170.0) * cos(fTheta) - (203.0) * sin(fTheta);
      float fObjectCenterY1 = (170.0) * sin(fTheta) + (203.0) * cos(fTheta);

      float fObjectCenterX2 = (170.0) * cos(fTheta) - (-203.0) * sin(fTheta);
      float fObjectCenterY2 = (170.0) * sin(fTheta) + (-203.0) * cos(fTheta);

      int iNumBase = vfClusterCenterY[iNumi] < vfClusterCenterY[iNumj] ? iNumi : iNumj;

      fObjectCenterX1 = fObjectCenterX1 + vfClusterCenterX[iNumBase];
      fObjectCenterY1 = fObjectCenterY1 + vfClusterCenterY[iNumBase];

      fObjectCenterX2 = fObjectCenterX2 + vfClusterCenterX[iNumBase];
      fObjectCenterY2 = fObjectCenterY2 + vfClusterCenterY[iNumBase];

      float fLength1 = get_length(fObjectCenterX1, fObjectCenterY1, fChairCenterXb1, fChairCenterYb1);
      float fLength2 = get_length(fObjectCenterX2, fObjectCenterY2, fChairCenterXb1, fChairCenterYb1);

      if (fLength1 < fLength2)
      {
        fObjectCenterX = fObjectCenterX1;
        fObjectCenterY = fObjectCenterY1;
      }
      else
      {
        fObjectCenterX = fObjectCenterX2;
        fObjectCenterY = fObjectCenterY2;
      }
    }
    else
    {
      fObjectCenterX = fChairCenterXb1;
      fObjectCenterY = fChairCenterYb1;
    }
  }
  else
  {
    fObjectCenterX = fChairCenterXb1;
    fObjectCenterY = fChairCenterYb1;

    // return false;
  }

  fChairCenterXb1 = fObjectCenterX;
  fChairCenterYb1 = fObjectCenterY;

  //--------------------------------------------------------------------------
  // Estimate four-point of rectangle(Wagon)
  float fDx = 0.0;
  float fDy = 0.0;

  vfVirtualPointX.clear();
  vfVirtualPointY.clear();

  if (iChairLeglNum == 4 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    for (int i = 0; i < 4; i++)
    {
      vfVirtualPointX.push_back(vfClusterCenterX[i]);
      vfVirtualPointY.push_back(vfClusterCenterY[i]);
    }
  }
  else if (iChairLeglNum == 3 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    for (unsigned int i = 0; i < 3; i++)
    {
      if (i != iNumi && i != iNumj)
      {
        fDx = (fObjectCenterX > vfClusterCenterX[i] ? fObjectCenterX - vfClusterCenterX[i] :
                                                      vfClusterCenterX[i] - fObjectCenterX);
        fDy = (fObjectCenterY > vfClusterCenterY[i] ? fObjectCenterY - vfClusterCenterY[i] :
                                                      vfClusterCenterY[i] - fObjectCenterY);

        if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * -1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * 1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * 1;
          fDy = fDy * 1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * -1;
          fDy = fDy * 1;
        }

        vfVirtualPointX.push_back(fObjectCenterX + fDx);
        vfVirtualPointY.push_back(fObjectCenterY + fDy);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
        printf("fDx = %f\n", fDx);
        printf("fDy = %f\n", fDy);
        printf("i = %d\n", i);
      }
      else
      {
        printf("i = %d\n", i);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
      }
    }
  }
  else if (iChairLeglNum == 2 && fObjectCenterX != 0 && fObjectCenterY != 0)
  {
    if (fLength_max >= fMinDiagonalLength)
    {
      vfVirtualPointX = vfChairBeforeVirtualPointX;
      vfVirtualPointY = vfChairBeforeVirtualPointY;
    }
    else if (fLength_max > fMinFrontLength && fLength_max < fMaxSideLength)
    {
      for (unsigned int i = 0; i < 2; i++)
      {
        fDx = (fObjectCenterX > vfClusterCenterX[i] ? fObjectCenterX - vfClusterCenterX[i] :
                                                      vfClusterCenterX[i] - fObjectCenterX);
        fDy = (fObjectCenterY > vfClusterCenterY[i] ? fObjectCenterY - vfClusterCenterY[i] :
                                                      vfClusterCenterY[i] - fObjectCenterY);

        if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * -1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY > 0)
        {
          fDx = fDx * 1;
          fDy = fDy * -1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX < 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * 1;
          fDy = fDy * 1;
        }
        else if (vfClusterCenterX[i] - fObjectCenterX > 0 && vfClusterCenterY[i] - fObjectCenterY < 0)
        {
          fDx = fDx * -1;
          fDy = fDy * 1;
        }

        vfVirtualPointX.push_back(fObjectCenterX + fDx);
        vfVirtualPointY.push_back(fObjectCenterY + fDy);
        vfVirtualPointX.push_back(vfClusterCenterX[i]);
        vfVirtualPointY.push_back(vfClusterCenterY[i]);
        printf("fDx = %f\n", fDx);
        printf("fDy = %f\n", fDy);
        printf("i = %d\n", i);
      }
    }
  }
  else
  {
    if (vfChairBeforeVirtualPointX.size() == 4 && vfChairBeforeVirtualPointY.size() == 4)
    {
      vfVirtualPointX = vfChairBeforeVirtualPointX;
      vfVirtualPointY = vfChairBeforeVirtualPointY;
    }
    else
    {
      for (unsigned int i = 0; i < 4; i++)
      {
        vfVirtualPointX.push_back(0.0);
        vfVirtualPointY.push_back(0.0);
      }
    }
  }

  vfChairBeforeVirtualPointX.clear();
  vfChairBeforeVirtualPointY.clear();
  vfChairBeforeVirtualPointX = vfVirtualPointX;
  vfChairBeforeVirtualPointY = vfVirtualPointY;

  //--------------------------------------------------------------------------
  // Sort virtual point
  if (vfVirtualPointX.size() == 4 && vfVirtualPointY.size() == 4)
  {
    //----------------------------------------------------------------------
    float fMaximum = 0.0;
    float fMinimum = 6000.0;
    float fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointY[i];

      if (fPosition > fMaximum)
      {
        iNumi = i;
        fMaximum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMinimum = 6000.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointX[i];

      if (fPosition < fMinimum)
      {
        iNumi = i;
        fMinimum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMinimum = 4000.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointY[i];

      if (fPosition < fMinimum)
      {
        iNumi = i;
        fMinimum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
    fMaximum = 0.0;
    fPosition = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
      fPosition = vfVirtualPointX[i];

      if (fPosition > fMaximum)
      {
        iNumi = i;
        fMaximum = fPosition;
      }
    }
    vfSortedVirtualPointX.push_back(vfVirtualPointX[iNumi]);
    vfSortedVirtualPointY.push_back(vfVirtualPointY[iNumi]);

    //----------------------------------------------------------------------
  }
  else
  {
    vfSortedVirtualPointX.clear();
    vfSortedVirtualPointY.clear();
    for (unsigned int i = 0; i < 4; i++)
    {
      vfSortedVirtualPointX.push_back(0.0);
      vfSortedVirtualPointY.push_back(0.0);
    }
  }

  //--------------------------------------------------------------------------
  // Publish message
  fss_chair_data.header.frame_id = "fss_chair_data";
  fss_chair_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_chair_data.tMeasuredTime = stClassData.tMeasuredTime;

  tms_msg_db::tmsdb_data msgTmsdbData;

  msgTmsdbData.tMeasuredTime = stClassData.tMeasuredTime;

  msgTmsdbData.iType = TYPE_MOVABLE_FUNITURE;
  msgTmsdbData.iID = ID_CHAIR1;

  msgTmsdbData.fX = fObjectCenterX;
  msgTmsdbData.fY = fObjectCenterY;
  msgTmsdbData.fZ = 0;
  msgTmsdbData.fTheta = 0;

  msgTmsdbData.iPlace = ID_FLOOR;
  msgTmsdbData.iState = STATE_EXIST;

  fss_chair_data.msgTMSInfo.push_back(msgTmsdbData);

  fss_chair_data.iGroupsCount = iGroupsCount;
  fss_chair_data.fClusterCenterX = vfClusterCenterX;
  fss_chair_data.fClusterCenterY = vfClusterCenterY;
  fss_chair_data.fClusterSize = vfClusterSize;
  fss_chair_data.fAvgIntrinsicIntensity = vfAvgIntrinsicIntensity;
  fss_chair_data.fVirtualPointX = vfSortedVirtualPointX;
  fss_chair_data.fVirtualPointY = vfSortedVirtualPointY;

  fss_chair_data.LrfData = vstDatas;

  rosPubChair.publish(fss_chair_data);

  stChairCenter.fX = fss_chair_data.msgTMSInfo[0].fX;
  stChairCenter.fY = fss_chair_data.msgTMSInfo[0].fY;

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed chairTracking #e");

  return true;
}

//------------------------------------------------------------------------------
void trackCallback(const tms_msg_ss::fss_class_data::ConstPtr &msg)
{
  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("trackCallback #s");

  //--------------------------------------------------------------------------
  CLASS_DATA stFssClassData;

  stFssClassData.tMeasuredTime = msg->tMeasuredTime;
  stFssClassData.viID = msg->iID;
  stFssClassData.vfClusterCenterX = msg->fCenterX;
  stFssClassData.vfClusterCenterY = msg->fCenterY;
  stFssClassData.vfClusterSize = msg->fSize;
  stFssClassData.vfAvgIntrinsicIntensity = msg->fAvgIntrinsicIntensity;
  stFssClassData.vstDatas = msg->LrfData;

  //--------------------------------------------------------------------------
  tms_msg_ss::fss_object_data fss_unknown_object_data;
  tms_msg_ss::fss_class_data fss_unknown_class_data;

  CLASS_DATA stSmartpal;
  CLASS_DATA stRoomba;
  CLASS_DATA stWagon;
  CLASS_DATA stChair;

  bool bIsExistSmartpalID = false;
  bool bIsExistRoombaID = false;
  bool bIsExistWagonID = false;
  bool bIsExistChairID = false;

  bool bIsExistSmartpal = false;
  bool bIsExistRoomba = false;
  bool bIsExistWagon = false;
  bool bIsExistChair = false;

  //--------------------------------------------------------------------------
  // tracking prc. of smartpal
  for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
  {
    if (stFssClassData.viID[i] == 1)  // class id = 1 , smartpal
    {
      stSmartpal.tMeasuredTime = stFssClassData.tMeasuredTime;
      stSmartpal.viID.push_back(stFssClassData.viID[i]);
      stSmartpal.vfClusterCenterX.push_back(stFssClassData.vfClusterCenterX[i]);
      stSmartpal.vfClusterCenterY.push_back(stFssClassData.vfClusterCenterY[i]);
      stSmartpal.vfClusterSize.push_back(stFssClassData.vfClusterSize[i]);
      stSmartpal.vfAvgIntrinsicIntensity.push_back(stFssClassData.vfAvgIntrinsicIntensity[i]);
      stSmartpal.vstDatas.push_back(stFssClassData.vstDatas[i]);
      bIsExistSmartpalID = true;
    }
  }

  if (bIsExistSmartpalID)
  {
    bIsExistSmartpal = smartpalTracking(stSmartpal);

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      if (stFssClassData.viID[i] == 1)  // class id = 1 , smartpal
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }

  if (bIsExistSmartpal)
  {
    ROS_INFO_STREAM("Smartpal : EXIST");

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      float fRadius = sqrt((stFssClassData.vfClusterCenterX[i] - stSmartpalCenter.fX) *
                               (stFssClassData.vfClusterCenterX[i] - stSmartpalCenter.fX) +
                           (stFssClassData.vfClusterCenterY[i] - stSmartpalCenter.fY) *
                               (stFssClassData.vfClusterCenterY[i] - stSmartpalCenter.fY));

      if (fRadius <= SMARTPAL_R)
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }
  else
  {
    //----------------------------------------------------------------------
    // Publish message
    tms_msg_ss::fss_object_data fss_smartpal_data;

    fss_smartpal_data.header.frame_id = "fss_smartpal_data";
    fss_smartpal_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_smartpal_data.tMeasuredTime = stFssClassData.tMeasuredTime;

    tms_msg_db::tmsdb_data msgTmsdbData;

    msgTmsdbData.tMeasuredTime = stFssClassData.tMeasuredTime;

    msgTmsdbData.iType = TYPE_ROBOT;
    msgTmsdbData.iID = ID_SMARTPAL;

    msgTmsdbData.fX = 0;
    msgTmsdbData.fY = 0;
    msgTmsdbData.fZ = 0;
    msgTmsdbData.fTheta = 0;

    msgTmsdbData.iPlace = ID_NONE;
    msgTmsdbData.iState = STATE_NOT_EXIST;

    fss_smartpal_data.msgTMSInfo.push_back(msgTmsdbData);

    fss_smartpal_data.iGroupsCount = 0;

    fss_smartpal_data.fClusterCenterX.clear();
    fss_smartpal_data.fClusterCenterY.clear();
    fss_smartpal_data.fClusterSize.clear();
    fss_smartpal_data.fAvgIntrinsicIntensity.clear();
    fss_smartpal_data.fVirtualPointX.clear();
    fss_smartpal_data.fVirtualPointY.clear();

    fss_smartpal_data.LrfData.clear();

    rosPubSmartpal.publish(fss_smartpal_data);
    ROS_INFO_STREAM("Smartpal : NOT_EXIST");
  }

  ROS_INFO_STREAM("end of smartpal trackCallback #e");

  //--------------------------------------------------------------------------
  // tracking prc. of roomba
  for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
  {
    if (stFssClassData.viID[i] == 2)  // class id = 2 , roomba
    {
      stRoomba.tMeasuredTime = stFssClassData.tMeasuredTime;
      stRoomba.viID.push_back(stFssClassData.viID[i]);
      stRoomba.vfClusterCenterX.push_back(stFssClassData.vfClusterCenterX[i]);
      stRoomba.vfClusterCenterY.push_back(stFssClassData.vfClusterCenterY[i]);
      stRoomba.vfClusterSize.push_back(stFssClassData.vfClusterSize[i]);
      stRoomba.vfAvgIntrinsicIntensity.push_back(stFssClassData.vfAvgIntrinsicIntensity[i]);
      stRoomba.vstDatas.push_back(stFssClassData.vstDatas[i]);
      bIsExistRoombaID = true;
    }
  }

  if (bIsExistRoombaID)
  {
    bIsExistRoomba = roombaTracking(stRoomba);

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      if (stFssClassData.viID[i] == 2)  // class id = 2 , roomba
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }

  if (bIsExistRoomba)
  {
    ROS_INFO_STREAM("Roomba : EXIST");

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      float fRadius = sqrt((stFssClassData.vfClusterCenterX[i] - stRoombaCenter.fX) *
                               (stFssClassData.vfClusterCenterX[i] - stRoombaCenter.fX) +
                           (stFssClassData.vfClusterCenterY[i] - stRoombaCenter.fY) *
                               (stFssClassData.vfClusterCenterY[i] - stRoombaCenter.fY));

      if (fRadius <= ROOMBA_R)
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }
  else
  {
    //----------------------------------------------------------------------
    // Publish fss_roomba_data message
    tms_msg_ss::fss_object_data fss_roomba_data;

    fss_roomba_data.header.frame_id = "fss_roomba_data";
    fss_roomba_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_roomba_data.tMeasuredTime = stFssClassData.tMeasuredTime;

    tms_msg_db::tmsdb_data msgTmsdbData;

    msgTmsdbData.tMeasuredTime = stFssClassData.tMeasuredTime;

    msgTmsdbData.iType = TYPE_ROBOT;
    msgTmsdbData.iID = ID_ROOMBA;

    msgTmsdbData.fX = 0;
    msgTmsdbData.fY = 0;
    msgTmsdbData.fZ = 0;
    msgTmsdbData.fTheta = 0;

    msgTmsdbData.iPlace = ID_NONE;
    msgTmsdbData.iState = STATE_NOT_EXIST;

    fss_roomba_data.msgTMSInfo.push_back(msgTmsdbData);

    fss_roomba_data.iGroupsCount = 0;

    fss_roomba_data.fClusterCenterX.clear();
    fss_roomba_data.fClusterCenterY.clear();
    fss_roomba_data.fClusterSize.clear();
    fss_roomba_data.fAvgIntrinsicIntensity.clear();
    fss_roomba_data.fVirtualPointX.clear();
    fss_roomba_data.fVirtualPointY.clear();

    fss_roomba_data.LrfData.clear();

    rosPubRoomba.publish(fss_roomba_data);
    ROS_INFO_STREAM("Roomba : NOT_EXIST");
  }

  ROS_INFO_STREAM("end of fss_roomba_data trackCallback #e");

  //--------------------------------------------------------------------------
  // tracking prc. of wagon
  for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
  {
    if (stFssClassData.viID[i] == 21)  // class id = 21 , wagon
    {
      stWagon.tMeasuredTime = stFssClassData.tMeasuredTime;
      stWagon.viID.push_back(stFssClassData.viID[i]);
      stWagon.vfClusterCenterX.push_back(stFssClassData.vfClusterCenterX[i]);
      stWagon.vfClusterCenterY.push_back(stFssClassData.vfClusterCenterY[i]);
      stWagon.vfClusterSize.push_back(stFssClassData.vfClusterSize[i]);
      stWagon.vfAvgIntrinsicIntensity.push_back(stFssClassData.vfAvgIntrinsicIntensity[i]);
      stWagon.vstDatas.push_back(stFssClassData.vstDatas[i]);
      bIsExistWagonID = true;
    }
  }

  // bIsExistWagonID = false;

  if (bIsExistWagonID)
  {
    bIsExistWagon = wagonTracking(stWagon);

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      if (stFssClassData.viID[i] == 21)  // class id = 21 , wagon
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }

  if (bIsExistWagon)
  {
    ROS_INFO_STREAM("Wagon : EXIST");

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      float fRadius = sqrt((stFssClassData.vfClusterCenterX[i] - stWagonCenter.fX) *
                               (stFssClassData.vfClusterCenterX[i] - stWagonCenter.fX) +
                           (stFssClassData.vfClusterCenterY[i] - stWagonCenter.fY) *
                               (stFssClassData.vfClusterCenterY[i] - stWagonCenter.fY));

      if (fRadius <= WAGON_R)
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }
  else
  {
    //----------------------------------------------------------------------
    // Publish fss_wagon_data message
    tms_msg_ss::fss_object_data fss_wagon_data;

    fss_wagon_data.header.frame_id = "fss_wagon_data";
    fss_wagon_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_wagon_data.tMeasuredTime = stFssClassData.tMeasuredTime;

    tms_msg_db::tmsdb_data msgTmsdbData;

    msgTmsdbData.tMeasuredTime = stFssClassData.tMeasuredTime;

    msgTmsdbData.iType = TYPE_MOVABLE_FUNITURE;
    msgTmsdbData.iID = ID_WAGON1;

    msgTmsdbData.fX = 0;
    msgTmsdbData.fY = 0;
    msgTmsdbData.fZ = 0;
    msgTmsdbData.fTheta = 0;

    msgTmsdbData.iPlace = ID_NONE;
    msgTmsdbData.iState = STATE_NOT_EXIST;

    fss_wagon_data.msgTMSInfo.push_back(msgTmsdbData);

    fss_wagon_data.iGroupsCount = 0;

    fss_wagon_data.fClusterCenterX.clear();
    fss_wagon_data.fClusterCenterY.clear();
    fss_wagon_data.fClusterSize.clear();
    fss_wagon_data.fAvgIntrinsicIntensity.clear();
    fss_wagon_data.fVirtualPointX.clear();
    fss_wagon_data.fVirtualPointY.clear();

    fss_wagon_data.LrfData.clear();

    rosPubWagon.publish(fss_wagon_data);
    ROS_INFO_STREAM("Wagon : NOT_EXIST");
  }

  ROS_INFO_STREAM("end of fss_wagon_data trackCallback #e");

  //--------------------------------------------------------------------------
  // tracking prc. of chair
  for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
  {
    printf("stFssClassData.viID[i] = %d \n", i);

    if (stFssClassData.viID[i] == 23)  // class id = 23 , chair
    {
      stChair.tMeasuredTime = stFssClassData.tMeasuredTime;
      stChair.viID.push_back(stFssClassData.viID[i]);
      stChair.vfClusterCenterX.push_back(stFssClassData.vfClusterCenterX[i]);
      stChair.vfClusterCenterY.push_back(stFssClassData.vfClusterCenterY[i]);
      stChair.vfClusterSize.push_back(stFssClassData.vfClusterSize[i]);
      stChair.vfAvgIntrinsicIntensity.push_back(stFssClassData.vfAvgIntrinsicIntensity[i]);
      stChair.vstDatas.push_back(stFssClassData.vstDatas[i]);
      bIsExistChairID = true;
    }
  }

  if (bIsExistChairID)
  {
    bIsExistChair = chairTracking(stChair);

    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      if (stFssClassData.viID[i] == 23)  // class id = 23 , chair
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }

  if (bIsExistChair)
  {
    ROS_INFO_STREAM("Chair : EXIST");
    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      float fRadius = sqrt((stFssClassData.vfClusterCenterX[i] - stChairCenter.fX) *
                               (stFssClassData.vfClusterCenterX[i] - stChairCenter.fX) +
                           (stFssClassData.vfClusterCenterY[i] - stChairCenter.fY) *
                               (stFssClassData.vfClusterCenterY[i] - stChairCenter.fY));

      if (fRadius <= CHAIR_R)
      {
        stFssClassData.viID.erase(stFssClassData.viID.begin() + i);
        stFssClassData.vfClusterCenterX.erase(stFssClassData.vfClusterCenterX.begin() + i);
        stFssClassData.vfClusterCenterY.erase(stFssClassData.vfClusterCenterY.begin() + i);
        stFssClassData.vfClusterSize.erase(stFssClassData.vfClusterSize.begin() + i);
        stFssClassData.vfAvgIntrinsicIntensity.erase(stFssClassData.vfAvgIntrinsicIntensity.begin() + i);
        stFssClassData.vstDatas.erase(stFssClassData.vstDatas.begin() + i);
        i--;
      }
    }
  }
  else
  {
    //----------------------------------------------------------------------
    // Publish fss_chair_data message
    tms_msg_ss::fss_object_data fss_chair_data;

    fss_chair_data.header.frame_id = "fss_chair_data";
    fss_chair_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
    fss_chair_data.tMeasuredTime = stFssClassData.tMeasuredTime;

    tms_msg_db::tmsdb_data msgTmsdbData;

    msgTmsdbData.tMeasuredTime = stFssClassData.tMeasuredTime;

    msgTmsdbData.iType = TYPE_MOVABLE_FUNITURE;
    msgTmsdbData.iID = ID_CHAIR1;

    msgTmsdbData.fX = 0;
    msgTmsdbData.fY = 0;
    msgTmsdbData.fZ = 0;
    msgTmsdbData.fTheta = 0;

    msgTmsdbData.iPlace = ID_NONE;
    msgTmsdbData.iState = STATE_NOT_EXIST;

    fss_chair_data.msgTMSInfo.push_back(msgTmsdbData);

    fss_chair_data.iGroupsCount = 0;

    fss_chair_data.fClusterCenterX.clear();
    fss_chair_data.fClusterCenterY.clear();
    fss_chair_data.fClusterSize.clear();
    fss_chair_data.fAvgIntrinsicIntensity.clear();
    fss_chair_data.fVirtualPointX.clear();
    fss_chair_data.fVirtualPointY.clear();

    fss_chair_data.LrfData.clear();

    rosPubChair.publish(fss_chair_data);
    ROS_INFO_STREAM("Chair : NOT_EXIST");
  }

  ROS_INFO_STREAM("end of fss_chair_data trackCallback #e");

  //--------------------------------------------------------------------------
  // occlusion process
  vector< uint32_t > viCurrentScanID;
  vector< float > vfCurrentDistance;
  vector< float > vfCurrentX;
  vector< float > vfCurrentY;
  vector< uint32_t > viBeforeScanID;
  vector< float > vfBeforeDistance;
  vector< float > vfBeforeX;
  vector< float > vfBeforeY;
  vector< uint32_t > viOcclusionScanID;
  vector< float > vfOcclusionDistance;
  vector< float > vfOcclusionX;
  vector< float > vfOcclusionY;
  POINT_GROUP stOcclusionGroupTemp;
  vector< POINT_GROUP > stOcclusionGroup;
  POINT_GROUP stCurrentGroupTemp;
  vector< POINT_GROUP > stCurrentGroup;
  POINT_GROUP stRestorationGroupTemp;
  vector< POINT_GROUP > stRestorationGroup;

  CLASS_DATA stReclusterDataTemp;
  CLASS_DATA stReclusterData;

  bool bIsIncludeOcclusionGroup = false;

  //--------------------------------------------------------------------------
  if (vstBefore_fss_unknown_cluster_data.size() >= 10)
    printf("vstBefore_fss_unknown_cluster_data[0].iID.size() = %d\n", vstBefore_fss_unknown_cluster_data[0].iID.size());

  printf("stFssClassData.viID.size() = %d\n", stFssClassData.viID.size());

  //--------------------------------------------------------------------------
  if (stFssClassData.viID.size() != 0)
  {
    //----------------------------------------------------------------------
    // Current scan point
    for (unsigned int i = 0; i < stFssClassData.viID.size(); i++)
    {
      for (unsigned int j = 0; j < stFssClassData.vstDatas[i].iScanID.size(); j++)
      {
        viCurrentScanID.push_back(stFssClassData.vstDatas[i].iScanID[j]);
        vfCurrentDistance.push_back(stFssClassData.vstDatas[i].fDistance[j]);
        vfCurrentX.push_back(stFssClassData.vstDatas[i].fX2[j]);
        vfCurrentY.push_back(stFssClassData.vstDatas[i].fY2[j]);
      }
    }

    //----------------------------------------------------------------------
    // Before scan point
    if (vstBefore_fss_unknown_cluster_data.size() >= 10)
    {
      for (unsigned int i = 0; i < vstBefore_fss_unknown_cluster_data[0].iID.size(); i++)
      {
        for (unsigned int j = 0; j < vstBefore_fss_unknown_cluster_data[0].LrfData[i].iScanID.size(); j++)
        {
          viBeforeScanID.push_back(vstBefore_fss_unknown_cluster_data[0].LrfData[i].iScanID[j]);
          vfBeforeDistance.push_back(vstBefore_fss_unknown_cluster_data[0].LrfData[i].fDistance[j]);
          vfBeforeX.push_back(vstBefore_fss_unknown_cluster_data[0].LrfData[i].fX2[j]);
          vfBeforeY.push_back(vstBefore_fss_unknown_cluster_data[0].LrfData[i].fY2[j]);
        }
      }
    }

    //----------------------------------------------------------------------
    // Avoid overlapping in before scan point
    for (unsigned int i = 0; i < viBeforeScanID.size(); i++)
    {
      for (unsigned int j = i + 1; j < viBeforeScanID.size(); j++)
      {
        if (viBeforeScanID[i] == viBeforeScanID[j])
        {
          if (vfBeforeDistance[i] >= vfBeforeDistance[j])
          {
            viBeforeScanID.erase(viBeforeScanID.begin() + j);
            vfBeforeDistance.erase(vfBeforeDistance.begin() + j);
            vfBeforeX.erase(vfBeforeX.begin() + j);
            vfBeforeY.erase(vfBeforeY.begin() + j);
            j--;
          }
          else
          {
            viBeforeScanID.erase(viBeforeScanID.begin() + i);
            vfBeforeDistance.erase(vfBeforeDistance.begin() + i);
            vfBeforeX.erase(vfBeforeX.begin() + i);
            vfBeforeY.erase(vfBeforeY.begin() + i);
            i--;
          }
        }
      }
    }

    //----------------------------------------------------------------------
    // Before scan point inside occlusion area
    for (unsigned int i = 0; i < viBeforeScanID.size(); i++)
    {
      for (unsigned int j = 0; j < viCurrentScanID.size(); j++)
      {
        if (viBeforeScanID[i] == viCurrentScanID[j])
        {
          if (vfBeforeDistance[i] - 50 > vfCurrentDistance[j])  // mm
          {
            viOcclusionScanID.push_back(viBeforeScanID[i]);
            vfOcclusionDistance.push_back(vfBeforeDistance[i]);
            vfOcclusionX.push_back(vfBeforeX[i]);
            vfOcclusionY.push_back(vfBeforeY[i]);
          }
        }
      }
    }

    //----------------------------------------------------------------------
    // Grouping of before scan point inside occlusion area
    for (unsigned int i = 0; i < viOcclusionScanID.size(); i++)
    {
      if (i + 1 == viOcclusionScanID.size())
      {
        stOcclusionGroupTemp.viScanID.push_back(viOcclusionScanID[i]);
        stOcclusionGroupTemp.vfDistance.push_back(vfOcclusionDistance[i]);
        stOcclusionGroupTemp.vfX.push_back(vfOcclusionX[i]);
        stOcclusionGroupTemp.vfY.push_back(vfOcclusionY[i]);

        stOcclusionGroup.push_back(stOcclusionGroupTemp);

        stOcclusionGroupTemp.viScanID.clear();
        stOcclusionGroupTemp.vfDistance.clear();
        stOcclusionGroupTemp.vfX.clear();
        stOcclusionGroupTemp.vfY.clear();
        break;
      }

      float fLength = get_length(vfOcclusionX[i], vfOcclusionY[i], vfOcclusionX[i + 1], vfOcclusionY[i + 1]);

      if (fLength < 50)  // mm
      {
        stOcclusionGroupTemp.viScanID.push_back(viOcclusionScanID[i]);
        stOcclusionGroupTemp.vfDistance.push_back(vfOcclusionDistance[i]);
        stOcclusionGroupTemp.vfX.push_back(vfOcclusionX[i]);
        stOcclusionGroupTemp.vfY.push_back(vfOcclusionY[i]);
      }
      else
      {
        stOcclusionGroupTemp.viScanID.push_back(viOcclusionScanID[i]);
        stOcclusionGroupTemp.vfDistance.push_back(vfOcclusionDistance[i]);
        stOcclusionGroupTemp.vfX.push_back(vfOcclusionX[i]);
        stOcclusionGroupTemp.vfY.push_back(vfOcclusionY[i]);

        stOcclusionGroup.push_back(stOcclusionGroupTemp);

        stOcclusionGroupTemp.viScanID.clear();
        stOcclusionGroupTemp.vfDistance.clear();
        stOcclusionGroupTemp.vfX.clear();
        stOcclusionGroupTemp.vfY.clear();
      }
    }
    printf("stOcclusionGroup = %d\n", stOcclusionGroup.size());

    //----------------------------------------------------------------------
    // Grouping of Current scan point inside occlusion area
    for (unsigned int i = 0; i < viCurrentScanID.size(); i++)
    {
      if (i + 1 == viCurrentScanID.size())
      {
        stCurrentGroupTemp.viScanID.push_back(viCurrentScanID[i]);
        stCurrentGroupTemp.vfDistance.push_back(vfCurrentDistance[i]);
        stCurrentGroupTemp.vfX.push_back(vfCurrentX[i]);
        stCurrentGroupTemp.vfY.push_back(vfCurrentY[i]);

        stCurrentGroup.push_back(stCurrentGroupTemp);

        stCurrentGroupTemp.viScanID.clear();
        stCurrentGroupTemp.vfDistance.clear();
        stCurrentGroupTemp.vfX.clear();
        stCurrentGroupTemp.vfY.clear();
        break;
      }

      float fLength = get_length(vfCurrentX[i], vfCurrentY[i], vfCurrentX[i + 1], vfCurrentY[i + 1]);

      if (fLength < 50)  // mm
      {
        stCurrentGroupTemp.viScanID.push_back(viCurrentScanID[i]);
        stCurrentGroupTemp.vfDistance.push_back(vfCurrentDistance[i]);
        stCurrentGroupTemp.vfX.push_back(vfCurrentX[i]);
        stCurrentGroupTemp.vfY.push_back(vfCurrentY[i]);
      }
      else
      {
        stCurrentGroupTemp.viScanID.push_back(viCurrentScanID[i]);
        stCurrentGroupTemp.vfDistance.push_back(vfCurrentDistance[i]);
        stCurrentGroupTemp.vfX.push_back(vfCurrentX[i]);
        stCurrentGroupTemp.vfY.push_back(vfCurrentY[i]);

        stCurrentGroup.push_back(stCurrentGroupTemp);

        stCurrentGroupTemp.viScanID.clear();
        stCurrentGroupTemp.vfDistance.clear();
        stCurrentGroupTemp.vfX.clear();
        stCurrentGroupTemp.vfY.clear();
      }
    }
    printf("stCurrentGroup = %d\n", stCurrentGroup.size());

    vector< POINT_GROUP > stStaticCurrentGroup;

    stStaticCurrentGroup = stCurrentGroup;

    //----------------------------------------------------------------------
    // re-grouping
    for (unsigned int i = 0; i < stOcclusionGroup.size(); i++)
    {
      bIsIncludeOcclusionGroup = false;
      int iRestorationNum = stOcclusionGroup[i].vfX.size();

      if (iRestorationNum != 0)
      {
        for (unsigned int j = 0; j < stCurrentGroup.size(); j++)
        {
          int iTagetNum = stCurrentGroup[j].vfX.size();

          float fLength1 = get_length(stOcclusionGroup[i].vfX[0], stOcclusionGroup[i].vfY[0], stCurrentGroup[j].vfX[0],
                                      stCurrentGroup[j].vfY[0]);
          float fLength2 = get_length(stOcclusionGroup[i].vfX[0], stOcclusionGroup[i].vfY[0],
                                      stCurrentGroup[j].vfX[iTagetNum - 1], stCurrentGroup[j].vfY[iTagetNum - 1]);
          float fLength3 =
              get_length(stOcclusionGroup[i].vfX[iRestorationNum - 1], stOcclusionGroup[i].vfY[iRestorationNum - 1],
                         stCurrentGroup[j].vfX[0], stCurrentGroup[j].vfY[0]);
          float fLength4 =
              get_length(stOcclusionGroup[i].vfX[iRestorationNum - 1], stOcclusionGroup[i].vfY[iRestorationNum - 1],
                         stCurrentGroup[j].vfX[iTagetNum - 1], stCurrentGroup[j].vfY[iTagetNum - 1]);

          if (fLength1 < 50 || fLength2 < 50 || fLength3 < 50 || fLength4 < 50)  // mm
          {
            if (!bIsIncludeOcclusionGroup)
            {
              for (unsigned int k = 0; k < stOcclusionGroup[i].viScanID.size(); k++)
              {
                stRestorationGroupTemp.viScanID.push_back(stOcclusionGroup[i].viScanID[k]);
                stRestorationGroupTemp.vfDistance.push_back(stOcclusionGroup[i].vfDistance[k]);
                stRestorationGroupTemp.vfX.push_back(stOcclusionGroup[i].vfX[k]);
                stRestorationGroupTemp.vfY.push_back(stOcclusionGroup[i].vfY[k]);
              }
              bIsIncludeOcclusionGroup = true;
            }

            for (unsigned int k = 0; k < stCurrentGroup[j].viScanID.size(); k++)
            {
              stRestorationGroupTemp.viScanID.push_back(stCurrentGroup[j].viScanID[k]);
              stRestorationGroupTemp.vfDistance.push_back(stCurrentGroup[j].vfDistance[k]);
              stRestorationGroupTemp.vfX.push_back(stCurrentGroup[j].vfX[k]);
              stRestorationGroupTemp.vfY.push_back(stCurrentGroup[j].vfY[k]);
            }
            stCurrentGroup.erase(stCurrentGroup.begin() + j);
            j--;
          }
          else
          {
            if (stOcclusionGroup[i].viScanID.size() >= 10)
            {
              for (uint32_t k = 0; k < stOcclusionGroup[i].viScanID.size(); k++)
              {
                stRestorationGroupTemp.viScanID.push_back(stOcclusionGroup[i].viScanID[k]);
                stRestorationGroupTemp.vfDistance.push_back(stOcclusionGroup[i].vfDistance[k]);
                stRestorationGroupTemp.vfX.push_back(stOcclusionGroup[i].vfX[k]);
                stRestorationGroupTemp.vfY.push_back(stOcclusionGroup[i].vfY[k]);
              }
            }
          }
        }

        if (stRestorationGroupTemp.viScanID.size() != 0)
        {
          stRestorationGroup.push_back(stRestorationGroupTemp);
        }

        stRestorationGroupTemp.viScanID.clear();
        stRestorationGroupTemp.vfDistance.clear();
        stRestorationGroupTemp.vfX.clear();
        stRestorationGroupTemp.vfY.clear();
      }
    }

    //----------------------------------------------------------------------
    // re-clusterring
    tms_msg_ss::fss_tf_datas vstDatas;

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

    uint32_t iGroupID = 201;
    float fCenterX = 0.0;
    float fCenterY = 0.0;

    //----------------------------------------------------------------------
    printf("after stRestorationGroup = %d\n", stRestorationGroup.size());
    for (unsigned int i = 0; i < stRestorationGroup.size(); i++)
    {
      //------------------------------------------------------------------
      for (unsigned int j = 0; j < stRestorationGroup[i].viScanID.size(); j++)
      {
        vstScanID.push_back(stRestorationGroup[i].viScanID[j]);
        vstReflect.push_back(false);        // tempValue
        vstIsForwardPoint.push_back(true);  // tempValue
        vstDistance.push_back(stRestorationGroup[i].vfDistance[j]);
        vstIntensity.push_back(0);           // tempValue
        vstIntrinsicIntensity.push_back(0);  // tempValue
        vstAcuteAngle.push_back(0);          // tempValue
        vstX1.push_back(0);                  // tempValue
        vstY1.push_back(0);                  // tempValue
        vstX2.push_back(stRestorationGroup[i].vfX[j]);
        vstY2.push_back(stRestorationGroup[i].vfY[j]);
      }
      //------------------------------------------------------------------
      stReclusterDataTemp.viID.push_back(iGroupID);

      //------------------------------------------------------------------
      fCenterX = fCenterY = 0.0;
      for (unsigned j = 0; j < vstX2.size(); j++)
      {
        fCenterX += vstX2[j];
        fCenterY += vstY2[j];
      }
      fCenterX = fCenterX / vstX2.size();
      fCenterY = fCenterY / vstY2.size();
      stReclusterDataTemp.vfClusterCenterX.push_back(fCenterX);
      stReclusterDataTemp.vfClusterCenterY.push_back(fCenterY);

      //------------------------------------------------------------------
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
      stReclusterDataTemp.vfClusterSize.push_back(fMax_size);

      //------------------------------------------------------------------
      stReclusterDataTemp.vfAvgIntrinsicIntensity.push_back(0);

      //------------------------------------------------------------------
      vstDatas.iScanID = vstScanID;
      vstDatas.bIsReflect = vstReflect;
      vstDatas.bIsForwardPoint = vstIsForwardPoint;
      vstDatas.fDistance = vstDistance;
      vstDatas.fIntensity = vstIntensity;
      vstDatas.fIntrinsicIntensity = vstIntrinsicIntensity;
      vstDatas.fAcuteAngle = vstAcuteAngle;
      vstDatas.fX1 = vstX1;
      vstDatas.fY1 = vstY1;
      vstDatas.fX2 = vstX2;
      vstDatas.fY2 = vstY2;

      stReclusterDataTemp.vstDatas.push_back(vstDatas);
      iGroupID++;

      //------------------------------------------------------------------
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
    printf("after stCurrentGroup = %d\n", stCurrentGroup.size());
    for (unsigned int i = 0; i < stCurrentGroup.size(); i++)
    {
      //------------------------------------------------------------------
      for (unsigned int j = 0; j < stCurrentGroup[i].viScanID.size(); j++)
      {
        vstScanID.push_back(stCurrentGroup[i].viScanID[j]);
        vstReflect.push_back(false);        // tempValue
        vstIsForwardPoint.push_back(true);  // tempValue
        vstDistance.push_back(stCurrentGroup[i].vfDistance[j]);
        vstIntensity.push_back(0);           // tempValue
        vstIntrinsicIntensity.push_back(0);  // tempValue
        vstAcuteAngle.push_back(0);          // tempValue
        vstX1.push_back(0);                  // tempValue
        vstY1.push_back(0);                  // tempValue
        vstX2.push_back(stCurrentGroup[i].vfX[j]);
        vstY2.push_back(stCurrentGroup[i].vfY[j]);
      }
      //------------------------------------------------------------------
      stReclusterDataTemp.viID.push_back(iGroupID);

      //------------------------------------------------------------------
      fCenterX = fCenterY = 0.0;
      for (unsigned j = 0; j < vstX2.size(); j++)
      {
        fCenterX += vstX2[j];
        fCenterY += vstY2[j];
      }
      fCenterX = fCenterX / vstX2.size();
      fCenterY = fCenterY / vstY2.size();
      stReclusterDataTemp.vfClusterCenterX.push_back(fCenterX);
      stReclusterDataTemp.vfClusterCenterY.push_back(fCenterY);

      //------------------------------------------------------------------
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
      stReclusterDataTemp.vfClusterSize.push_back(fMax_size);

      //------------------------------------------------------------------
      stReclusterDataTemp.vfAvgIntrinsicIntensity.push_back(0);

      //------------------------------------------------------------------
      vstDatas.iScanID = vstScanID;
      vstDatas.bIsReflect = vstReflect;
      vstDatas.bIsForwardPoint = vstIsForwardPoint;
      vstDatas.fDistance = vstDistance;
      vstDatas.fIntensity = vstIntensity;
      vstDatas.fIntrinsicIntensity = vstIntrinsicIntensity;
      vstDatas.fAcuteAngle = vstAcuteAngle;
      vstDatas.fX1 = vstX1;
      vstDatas.fY1 = vstY1;
      vstDatas.fX2 = vstX2;
      vstDatas.fY2 = vstY2;

      stReclusterDataTemp.vstDatas.push_back(vstDatas);
      iGroupID++;

      //------------------------------------------------------------------
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
    for (uint32_t i = 0; i < stReclusterDataTemp.viID.size(); i++)
    {
      for (uint32_t j = i + 1; j < stReclusterDataTemp.viID.size(); j++)
      {
        float fLength = 0.0;

        fLength = get_length(stReclusterDataTemp.vfClusterCenterX[i], stReclusterDataTemp.vfClusterCenterY[i],
                             stReclusterDataTemp.vfClusterCenterX[j], stReclusterDataTemp.vfClusterCenterY[j]);

        //--------------------------------------------------------------
        if (fLength < 200)
        {
          fCenterX = (stReclusterDataTemp.vfClusterCenterX[i] + stReclusterDataTemp.vfClusterCenterX[j]) / 2;

          fCenterY = (stReclusterDataTemp.vfClusterCenterY[i] + stReclusterDataTemp.vfClusterCenterY[j]) / 2;

          //----------------------------------------------------------
          for (uint32_t k = 0; k < stReclusterDataTemp.vstDatas[j].fX2.size(); k++)
          {
            stReclusterDataTemp.vstDatas[i].iScanID.push_back(stReclusterDataTemp.vstDatas[j].iScanID[k]);
            stReclusterDataTemp.vstDatas[i].bIsReflect.push_back(stReclusterDataTemp.vstDatas[j].bIsReflect[k]);
            stReclusterDataTemp.vstDatas[i].bIsForwardPoint.push_back(
                stReclusterDataTemp.vstDatas[j].bIsForwardPoint[k]);
            stReclusterDataTemp.vstDatas[i].fDistance.push_back(stReclusterDataTemp.vstDatas[j].fDistance[k]);
            stReclusterDataTemp.vstDatas[i].fIntensity.push_back(stReclusterDataTemp.vstDatas[j].fIntensity[k]);
            stReclusterDataTemp.vstDatas[i].fIntrinsicIntensity.push_back(
                stReclusterDataTemp.vstDatas[j].fIntrinsicIntensity[k]);
            stReclusterDataTemp.vstDatas[i].fAcuteAngle.push_back(stReclusterDataTemp.vstDatas[j].fAcuteAngle[k]);
            stReclusterDataTemp.vstDatas[i].fX1.push_back(stReclusterDataTemp.vstDatas[j].fX1[k]);
            stReclusterDataTemp.vstDatas[i].fY1.push_back(stReclusterDataTemp.vstDatas[j].fY1[k]);
            stReclusterDataTemp.vstDatas[i].fX2.push_back(stReclusterDataTemp.vstDatas[j].fX2[k]);
            stReclusterDataTemp.vstDatas[i].fY2.push_back(stReclusterDataTemp.vstDatas[j].fY2[k]);
          }

          stReclusterDataTemp.viID.erase(stReclusterDataTemp.viID.begin() + j);
          stReclusterDataTemp.vfClusterCenterX.erase(stReclusterDataTemp.vfClusterCenterX.begin() + j);
          stReclusterDataTemp.vfClusterCenterY.erase(stReclusterDataTemp.vfClusterCenterY.begin() + j);
          stReclusterDataTemp.vfClusterSize.erase(stReclusterDataTemp.vfClusterSize.begin() + j);
          stReclusterDataTemp.vfAvgIntrinsicIntensity.erase(stReclusterDataTemp.vfAvgIntrinsicIntensity.begin() + j);
          stReclusterDataTemp.vstDatas.erase(stReclusterDataTemp.vstDatas.begin() + j);

          j--;

          //----------------------------------------------------------
          float fMax_size = 0.0;
          float fLength = 0.0;
          for (unsigned l = 0; l < stReclusterDataTemp.vstDatas[i].fX2.size(); l++)
          {
            for (unsigned m = l; m < stReclusterDataTemp.vstDatas[i].fX2.size(); m++)
            {
              fLength = get_length(stReclusterDataTemp.vstDatas[i].fX2[l], stReclusterDataTemp.vstDatas[i].fY2[l],
                                   stReclusterDataTemp.vstDatas[i].fX2[m], stReclusterDataTemp.vstDatas[i].fY2[m]);

              if (fLength >= fMax_size)
              {
                fMax_size = fLength;
              }
            }
          }
          //----------------------------------------------------------
          stReclusterDataTemp.vfClusterCenterX[i] = fCenterX;
          stReclusterDataTemp.vfClusterCenterY[i] = fCenterY;
          stReclusterDataTemp.vfClusterSize[i] = fMax_size;

          //----------------------------------------------------------
        }
      }
    }
  }

  printf("id setting\n");
  //--------------------------------------------------------------------------
  // id setting
  uint32_t uiCountIndex = 0;
  int32_t iEndIndex = 0;
  bool bIDTag = false;

  if (vstBefore_fss_unknown_cluster_data.size() != 0)
  {
    tms_msg_ss::fss_class_data msgIdInformation;

    iEndIndex = vstBefore_fss_unknown_cluster_data.size() - 1;

    for (uint32_t i = 0; i < vstBefore_fss_unknown_cluster_data[iEndIndex].iID.size(); i++)
    {
      msgIdInformation.iID.push_back(vstBefore_fss_unknown_cluster_data[iEndIndex].iID[i]);
      msgIdInformation.fCenterX.push_back(vstBefore_fss_unknown_cluster_data[iEndIndex].fCenterX[i]);
      msgIdInformation.fCenterY.push_back(vstBefore_fss_unknown_cluster_data[iEndIndex].fCenterY[i]);
    }

    printf("t1\n");
    for (uint32_t i = 0; i < stReclusterDataTemp.viID.size(); i++)
    {
      float fMinLength = 10000.0;
      float fLength = 0.0;
      uint32_t uiMinLengthID = 0;

      for (uint32_t j = 0; j < msgIdInformation.iID.size(); j++)
      {
        fLength = get_length(stReclusterDataTemp.vfClusterCenterX[i], stReclusterDataTemp.vfClusterCenterY[i],
                             msgIdInformation.fCenterX[j], msgIdInformation.fCenterY[j]);

        if (fLength <= fMinLength)
        {
          fMinLength = fLength;
          uiMinLengthID = j;
        }

        bIDTag = true;
      }

      if (bIDTag == true)
      {
        stReclusterDataTemp.viID[i] = msgIdInformation.iID[uiMinLengthID];

        msgIdInformation.iID.erase(msgIdInformation.iID.begin() + uiMinLengthID);
        msgIdInformation.fCenterX.erase(msgIdInformation.fCenterX.begin() + uiMinLengthID);
        msgIdInformation.fCenterY.erase(msgIdInformation.fCenterY.begin() + uiMinLengthID);
      }
      else
      {
        stReclusterDataTemp.viID[i] = 301 + uiCountIndex;
        uiCountIndex++;
      }
      bIDTag = false;
    }
  }

  uiCountIndex = 201;

  for (uint32_t i = 0; i < stReclusterDataTemp.viID.size(); i++)
  {
    if (stReclusterDataTemp.viID[i] > 300)
    {
      bool bMatchingFlag = false;

      while (1)
      {
        for (uint32_t j = 0; j < stReclusterDataTemp.viID.size(); j++)
        {
          if (stReclusterDataTemp.viID[j] == uiCountIndex)
            bMatchingFlag = true;
        }

        if (bMatchingFlag)
        {
          uiCountIndex++;
          bMatchingFlag = false;
        }
        else
        {
          stReclusterDataTemp.viID[i] = uiCountIndex;
          uiCountIndex++;
          break;
        }
      }
    }
  }
  //--------------------------------------------------------------------------
  fss_unknown_class_data.header.frame_id = "fss_unknown_class_data";
  fss_unknown_class_data.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  fss_unknown_class_data.tMeasuredTime = stFssClassData.tMeasuredTime;

  fss_unknown_class_data.iID = stReclusterDataTemp.viID;
  fss_unknown_class_data.fCenterX = stReclusterDataTemp.vfClusterCenterX;
  fss_unknown_class_data.fCenterY = stReclusterDataTemp.vfClusterCenterY;
  fss_unknown_class_data.fSize = stReclusterDataTemp.vfClusterSize;
  fss_unknown_class_data.fAvgIntrinsicIntensity = stReclusterDataTemp.vfAvgIntrinsicIntensity;
  fss_unknown_class_data.LrfData = stReclusterDataTemp.vstDatas;

  fss_unknown_class_data.fOcclusionX = msg->fOcclusionX;
  fss_unknown_class_data.fOcclusionY = msg->fOcclusionY;

  rosPubUnknownClass.publish(fss_unknown_class_data);

  //--------------------------------------------------------------------------
  // save fss_unknown_class_data because occlusion process

  vstBefore_fss_unknown_cluster_data.push_back(fss_unknown_class_data);

  if (vstBefore_fss_unknown_cluster_data.size() >= 11)
    vstBefore_fss_unknown_cluster_data.erase(vstBefore_fss_unknown_cluster_data.begin());

  //--------------------------------------------------------------------------
  ROS_INFO_STREAM("completed sending of fss_object_data #e");
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  // ros setting
  ros::init(argc, argv, "fss_track");
  ros::NodeHandle nh;

  rosSub = nh.subscribe("fss_class_data", 10, trackCallback);

  rosPubSmartpal = nh.advertise< tms_msg_ss::fss_object_data >("fss_smartpal_data", 10);
  rosPubRoomba = nh.advertise< tms_msg_ss::fss_object_data >("fss_roomba_data", 10);
  rosPubWagon = nh.advertise< tms_msg_ss::fss_object_data >("fss_wagon_data", 10);
  rosPubChair = nh.advertise< tms_msg_ss::fss_object_data >("fss_chair_data", 10);

  rosPubUnknownClass = nh.advertise< tms_msg_ss::fss_class_data >("fss_unknown_class_data", 10);

  //--------------------------------------------------------------------------
  // ros spin
  ros::spin();

  return (0);
}

//------------------------------------------------------------------------------
