/*
 * Cluster.h
 *
 *  Created on: 2012/11/01
 *      Author: Masahide Tanaka
 */

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <time.h>
#include <float.h>
#include <vector>
#include <ros/ros.h>
#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_ss/fss_tf_datas.h>

class Cluster
{
public:
  Cluster();
  ~Cluster();

  //--------------------------------------------------------------------------
  // variables

  int iID;
  int iType;
  float fLikelihood;  // likelihood of person
  float fSizeMax;
  float fSizeMin;
  bool isUpdate;
  bool inOcclusionArea;
  double sensingTime;

  std::vector< int > iNumPoint;
  std::vector< float > fSize;
  std::vector< float > fCenterX;
  std::vector< float > fCenterY;
  std::vector< float > fMiddleX;
  std::vector< float > fMiddleY;
  std::vector< float > fRectangleMinX;
  std::vector< float > fRectangleMinY;
  std::vector< float > fRectangleMaxX;
  std::vector< float > fRectangleMaxY;

  std::vector< ros::Time > tMeasuredTime;
  std::vector< tms_msg_ss::fss_tf_datas > LrfData;

  //--------------------------------------------------------------------------
  // functions
  float fAveCenterX();
  float fAveCenterY();
  void copy(std::list< Cluster >::iterator it, int startIndex, int size, bool isFull = false);
  void update(const tms_msg_ss::fss_class_data::ConstPtr& msg, int index);
  void initialize(const tms_msg_ss::fss_class_data::ConstPtr& msg, int index);
};

#endif /* CLUSTER_H_ */
