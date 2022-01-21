/*
 * Cluster.cpp
 *
 *  Created on: 2012/11/01
 *      Author: Masahide Tanaka
 */

#include "Cluster.h"

Cluster::Cluster()
{
  iID = 0;
  iType = 0;
  fLikelihood = 0.0;
  sensingTime = 0.0;
  fSizeMax = 0.0;
  fSizeMin = 0.0;
  isUpdate = false;
  inOcclusionArea = false;
}

Cluster::~Cluster()
{
}

void Cluster::initialize(const tms_msg_ss::fss_class_data::ConstPtr& msg, int index)
{
  iID = msg->iID[index];
  iType = 0;
  fLikelihood = 0.5;  // Footprint probability at initializing: 50%
  sensingTime = 0.0;
  fSizeMax = msg->fSize[index];
  fSizeMin = msg->fSize[index];

  iNumPoint.push_back(msg->LrfData[index].fX2.size());
  fSize.push_back(msg->fSize[index]);
  fCenterX.push_back(msg->fCenterX[index]);
  fCenterY.push_back(msg->fCenterY[index]);
  tMeasuredTime.push_back(msg->tMeasuredTime);

  LrfData.push_back(msg->LrfData[index]);

  float tempRectangleMinX, tempRectangleMinY;
  float tempRectangleMaxX, tempRectangleMaxY;

  tempRectangleMinX = FLT_MAX;
  tempRectangleMinY = FLT_MAX;
  tempRectangleMaxX = FLT_MIN;
  tempRectangleMaxY = FLT_MIN;

  for (unsigned int j = 0; j < LrfData.back().fX2.size(); j++)
  {
    float x = LrfData.back().fX2[j];
    float y = LrfData.back().fY2[j];

    if (x < tempRectangleMinX)
      tempRectangleMinX = x;
    if (x > tempRectangleMaxX)
      tempRectangleMaxX = x;
    if (y < tempRectangleMinY)
      tempRectangleMinY = y;
    if (y > tempRectangleMaxY)
      tempRectangleMaxY = y;
  }
  fRectangleMinX.push_back(tempRectangleMinX);
  fRectangleMinY.push_back(tempRectangleMinY);
  fRectangleMaxX.push_back(tempRectangleMaxX);
  fRectangleMaxY.push_back(tempRectangleMaxY);

  fMiddleX.push_back((tempRectangleMinX + tempRectangleMaxX) / 2.0);
  fMiddleY.push_back((tempRectangleMinY + tempRectangleMaxY) / 2.0);
}

void Cluster::update(const tms_msg_ss::fss_class_data::ConstPtr& msg, int index)
{
  iID = msg->iID[index];

  if (fSizeMax < msg->fSize[index])
  {
    fSizeMax = msg->fSize[index];
  }
  if (fSizeMin > msg->fSize[index])
    fSizeMin = msg->fSize[index];

  sensingTime = (double)(msg->tMeasuredTime.toNSec() / tMeasuredTime.front().toNSec()) / (double)1.0e9;

  iNumPoint.push_back(msg->LrfData[index].fX2.size());
  fSize.push_back(msg->fSize[index]);
  fCenterX.push_back(msg->fCenterX[index]);
  fCenterY.push_back(msg->fCenterY[index]);
  tMeasuredTime.push_back(msg->tMeasuredTime);

  LrfData.push_back(msg->LrfData[index]);

  float tempRectangleMinX, tempRectangleMinY;
  float tempRectangleMaxX, tempRectangleMaxY;

  tempRectangleMinX = FLT_MAX;
  tempRectangleMinY = FLT_MAX;
  tempRectangleMaxX = FLT_MIN;
  tempRectangleMaxY = FLT_MIN;

  for (unsigned int j = 0; j < LrfData.back().fX2.size(); j++)
  {
    float x = LrfData.back().fX2[j];
    float y = LrfData.back().fY2[j];

    if (x < tempRectangleMinX)
      tempRectangleMinX = x;
    if (x > tempRectangleMaxX)
      tempRectangleMaxX = x;
    if (y < tempRectangleMinY)
      tempRectangleMinY = y;
    if (y > tempRectangleMaxY)
      tempRectangleMaxY = y;
  }
  fRectangleMinX.push_back(tempRectangleMinX);
  fRectangleMinY.push_back(tempRectangleMinY);
  fRectangleMaxX.push_back(tempRectangleMaxX);
  fRectangleMaxY.push_back(tempRectangleMaxY);

  fMiddleX.push_back((tempRectangleMinX + tempRectangleMaxX) / 2.0);
  fMiddleY.push_back((tempRectangleMinY + tempRectangleMaxY) / 2.0);

  isUpdate = true;
}

void Cluster::copy(std::list< Cluster >::iterator it, int startIndex, int size, bool isFull)
{
  if (isFull)
  {
    iID = it->iID;
    iType = it->iType;
    fLikelihood = it->fLikelihood;
    sensingTime = it->sensingTime;
    fSizeMax = it->fSizeMax;
    fSizeMin = it->fSizeMin;

    iNumPoint = it->iNumPoint;
    fSize = it->fSize;
    fCenterX = it->fCenterX;
    fCenterY = it->fCenterY;
    tMeasuredTime = it->tMeasuredTime;

    LrfData = it->LrfData;
    inOcclusionArea = it->inOcclusionArea;

    fRectangleMinX = it->fRectangleMinX;
    fRectangleMinY = it->fRectangleMinY;
    fRectangleMaxX = it->fRectangleMaxX;
    fRectangleMaxY = it->fRectangleMaxY;

    fMiddleX = it->fMiddleX;
    fMiddleY = it->fMiddleY;
  }
  else
  {
    iID = it->iID;
    iType = it->iType;
    fLikelihood = it->fLikelihood;
    sensingTime =
        (double)(it->tMeasuredTime[size - 1].toNSec() / it->tMeasuredTime[startIndex].toNSec()) / (double)1.0e9;
    inOcclusionArea = it->inOcclusionArea;

    float _fSizeMax = FLT_MIN;
    float _fSizeMin = FLT_MAX;
    for (int i = startIndex; i < size; i++)
    {
      if (it->fSize[i] > _fSizeMax)
        _fSizeMax = it->fSize[i];

      if (it->fSize[i] < _fSizeMin)
        _fSizeMin = it->fSize[i];

      iNumPoint.push_back(it->iNumPoint[i]);
      fSize.push_back(it->fSize[i]);
      fCenterX.push_back(it->fCenterX[i]);
      fCenterY.push_back(it->fCenterY[i]);
      tMeasuredTime.push_back(it->tMeasuredTime[i]);

      LrfData.push_back(it->LrfData[i]);

      fRectangleMinX.push_back(it->fRectangleMinX[i]);
      fRectangleMinY.push_back(it->fRectangleMinY[i]);
      fRectangleMaxX.push_back(it->fRectangleMaxX[i]);
      fRectangleMaxY.push_back(it->fRectangleMaxY[i]);

      fMiddleX.push_back(it->fMiddleX[i]);
      fMiddleY.push_back(it->fMiddleY[i]);
    }
    fSizeMax = _fSizeMax;
    fSizeMin = _fSizeMin;
  }
}

float Cluster::fAveCenterX()
{
  float ave_x = 0;
  for (unsigned int i = 0; i < fCenterX.size(); i++)
    ave_x += fCenterX[i];

  return ave_x / (float)fCenterX.size();
}

float Cluster::fAveCenterY()
{
  float ave_y = 0;
  for (unsigned int i = 0; i < fCenterY.size(); i++)
    ave_y += fCenterY[i];

  return ave_y / (float)fCenterY.size();
}
