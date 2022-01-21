//------------------------------------------------------------------------------
// @file   : viewer.cpp
// @brief  : plot object on xy-coordinate and display lrf-data-table
// @author : Yoonseok Pyo, Masahide Tanaka, Kohei Nakashima
// @version: Ver0.9.6.0 (since 2012.05.17)
// @date   : 2014.04.09
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui>
#include <QPainter>

#include <vector>
#include <float.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "viewer.h"
#include "qnode.hpp"

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
void Viewer::fssPreDataCallback(const tms_msg_ss::fss_pre_data::ConstPtr& msg)
{
}

//------------------------------------------------------------------------------
void Viewer::fssTfDataCallback(const tms_msg_ss::fss_tf_data::ConstPtr& msg)
{
  m_msgRawdata = *msg;

  for (unsigned int i = 0; i < m_msgRawdata.fDistance.size(); i++)
  {
    m_dUtm30LX_raw_data[i].fDistance = m_msgRawdata.fDistance[i];
    m_dUtm30LX_raw_data[i].fIntensity = m_msgRawdata.fIntensity[i];
    m_dUtm30LX_raw_data[i].fIntrinsicIntensity = m_msgRawdata.fIntrinsicIntensity[i];
    m_dUtm30LX_raw_data[i].fAcuteAngle = m_msgRawdata.fAcuteAngle[i];
  }
}

//------------------------------------------------------------------------------
void Viewer::fssTfDataCallback1(const tms_msg_ss::fss_tf_data::ConstPtr& msg)
{
  m_msgRawdata1 = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssTfDataCallback2(const tms_msg_ss::fss_tf_data::ConstPtr& msg)
{
  m_msgRawdata2 = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssClusterDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg)
{
  m_msgCluster = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssClassDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg)
{
  m_msgClass = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssUnknownClassDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg)
{
  m_msgUnknownClass = *msg;
}

//------------------------------------------------------------------------------
void Viewer::rpsPlanPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  m_msgPlanPath = *msg;
}

//------------------------------------------------------------------------------
void Viewer::rpsRobotPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  m_msgRobotPath = *msg;
}

//------------------------------------------------------------------------------
void Viewer::rpsWagonPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  m_msgWagonPath = *msg;
}

//------------------------------------------------------------------------------
void Viewer::RPS_MAPCallback(const tms_msg_rp::rps_map_full::ConstPtr& msg)
{
  m_msgRPS_MAP = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssSmartpalDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg)
{
  m_msgSmartpal = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssRoombaDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg)
{
  m_msgRoomba = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssWagonDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg)
{
  m_msgWagon = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssChairDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg)
{
  m_msgChair = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssPersonTrajectoryDataCallback(const tms_msg_ss::fss_person_trajectory_data::ConstPtr& msg)
{
  m_msgPersonTrajectory = *msg;
}

//------------------------------------------------------------------------------
void Viewer::fssUnknownObjectDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg)
{
  m_msgUnknownObject = *msg;
}

//------------------------------------------------------------------------------
void Viewer::issWagonDataCallback(const tms_msg_ss::iss_object_data::ConstPtr& msg)
{
  m_msgIssWagon = *msg;
}

//------------------------------------------------------------------------------
void Viewer::issWheelChairDataCallback(const tms_msg_ss::iss_object_data::ConstPtr& msg)
{
  m_msgIssWheelChair = *msg;
}

//------------------------------------------------------------------------------
void Viewer::icsObjectDataCallback(const tms_msg_ss::ics_object_data::ConstPtr& msg)
{
  m_msgIcsObjectData = *msg;
}

//------------------------------------------------------------------------------
Viewer::Viewer(QNode* node, QWidget* parent) : QWidget(parent), qnode(node)
{
  qColorID[1] = qRgb(0, 0, 0);         // black
  qColorID[2] = qRgb(0, 255, 255);     // cyan
  qColorID[3] = qRgb(255, 165, 0);     // orange
  qColorID[4] = qRgb(0, 0, 255);       // blue
  qColorID[5] = qRgb(0, 255, 0);       // green
  qColorID[6] = qRgb(189, 183, 107);   // DarkKhaki
  qColorID[7] = qRgb(0, 0, 128);       // Navy
  qColorID[8] = qRgb(238, 130, 238);   // violet
  qColorID[9] = qRgb(190, 190, 190);   // grey
  qColorID[10] = qRgb(139, 0, 0);      // darkRed
  qColorID[11] = qRgb(139, 69, 0);     // DarkOrange
  qColorID[12] = qRgb(139, 101, 8);    // DarkGoldenrod
  qColorID[13] = qRgb(65, 105, 225);   // RoyalBlue
  qColorID[14] = qRgb(255, 0, 0);      // red
  qColorID[15] = qRgb(255, 215, 0);    // gold
  qColorID[16] = qRgb(202, 255, 112);  // DarkOliveGreen1
  qColorID[17] = qRgb(240, 128, 128);  // LightCoral
  qColorID[18] = qRgb(255, 20, 147);   // DeepPink
  qColorID[19] = qRgb(153, 50, 204);   // DarkOrchid
  qColorID[20] = qRgb(245, 222, 179);  // wheat

  m_sObjectName[ID_NAMACHA] = "namach";
  m_sObjectName[ID_CHIPSTAR1] = "chipstar1";
  m_sObjectName[ID_CHIPSTAR2] = "chipstar2";
  m_sObjectName[ID_CHIPSTAR3] = "chipstar3";
  m_sObjectName[ID_BOTTLE] = "bottle";
  m_sObjectName[ID_COINBANK] = "coinbank";
  m_sObjectName[ID_WATER] = "water";
  m_sObjectName[ID_PETBOTTLE] = "petbottle";
  m_sObjectName[ID_BOOK] = "book";
  m_sObjectName[ID_CALPIS] = "calpis";

  m_viewer_scale = 0.150;

  m_dOffset_x = 100;
  m_dOffset_y = 100;

  m_iSelect_object = 0;

  m_iCallback = 0;
  m_bIs_background = false;

  m_iDataViewRowIndex = 0;

  init_lrf();

  load_map();
  load_grid();

  // ros setting
  ros::init(qnode->init_argc, qnode->init_argv, qnode->node_name);
  if (!ros::master::check())
  {
    return;
  }
  ros::NodeHandle nh;

  // fss_pre_data_subscriber               = nh.subscribe("fss_pre_data",      10, &Viewer::fssPreDataCallback, this);
  fss_tf_data_subscriber = nh.subscribe("fss_tf_data", 10, &Viewer::fssTfDataCallback, this);
  fss_tf_data_subscriber1 = nh.subscribe("fss_tf_data1", 10, &Viewer::fssTfDataCallback1, this);
  fss_tf_data_subscriber2 = nh.subscribe("fss_tf_data2", 10, &Viewer::fssTfDataCallback2, this);
  fss_cluster_data_subscriber = nh.subscribe("fss_cluster_data", 10, &Viewer::fssClusterDataCallback, this);
  fss_class_data_subscriber = nh.subscribe("fss_class_data", 10, &Viewer::fssClassDataCallback, this);
  fss_Unknown_class_data_subscriber =
      nh.subscribe("fss_unknown_class_data", 10, &Viewer::fssUnknownClassDataCallback, this);
  fss_smartpal_data_subscriber = nh.subscribe("fss_smartpal_data", 10, &Viewer::fssSmartpalDataCallback, this);
  fss_roomba_data_subscriber = nh.subscribe("fss_roomba_data", 10, &Viewer::fssRoombaDataCallback, this);
  fss_wagon_data_subscriber = nh.subscribe("fss_wagon_data", 10, &Viewer::fssWagonDataCallback, this);
  fss_chair_data_subscriber = nh.subscribe("fss_chair_data", 10, &Viewer::fssChairDataCallback, this);
  fss_unknown_object_data_subscriber =
      nh.subscribe("fss_unknown_object_data", 10, &Viewer::fssUnknownObjectDataCallback, this);
  iss_wagon_data_subscriber = nh.subscribe("iss_wagon_data", 10, &Viewer::issWagonDataCallback, this);
  iss_wheelchair_data_subscriber = nh.subscribe("iss_wheelchair_data", 10, &Viewer::issWheelChairDataCallback, this);
  ics_object_data_subscriber = nh.subscribe("ics_object_data", 10, &Viewer::icsObjectDataCallback, this);
  fss_person_trajectory_data_subscriber =
      nh.subscribe("fss_person_trajectory_data", 10, &Viewer::fssPersonTrajectoryDataCallback, this);

  rps_plan_path_subscriber = nh.subscribe("rps_plan_path", 10, &Viewer::rpsPlanPathCallback, this);
  rps_robot_path_subscriber = nh.subscribe("rps_robot_path", 10, &Viewer::rpsRobotPathCallback, this);
  rps_wagon_path_subscriber = nh.subscribe("rps_wagon_path", 10, &Viewer::rpsWagonPathCallback, this);
  rps_MAP_subscriber = nh.subscribe("rps_map_data", 10, &Viewer::RPS_MAPCallback, this);

  rosclientTmsdbRobotTrajectoryData =
      nh.serviceClient< tms_msg_db::tmsdb_robot_trajectory_data >("tmsdb_robot_trajectory_data");

  setMouseTracking(true);

  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(view()));
  timer->start(1);
}

//------------------------------------------------------------------------------
void Viewer::view()
{
  display();
  ros::spinOnce();
}

//------------------------------------------------------------------------------
void Viewer::display()
{
  //--------------------------------------------------------------------------
  int iPointSize = (int)(m_viewer_scale * 15);

  m_dWindow_size_x = width();
  m_dWindow_size_y = height();

  m_stImage.fill(qRgb(255, 255, 255));

  //--------------------------------------------------------------------------
  drawMap();

  if (m_iModeGrid == MODE_GRID_SQUARE)
    drawGridSquare();
  else if (m_iModeGrid == MODE_GRID_CIRCLE)
    drawGridCircle();

  //--------------------------------------------------------------------------
  if (m_msgRawdata.bIsReflect.size() != 0)
    viewLaser();

  //--------------------------------------------------------------------------
  if (m_msgRawdata1.bIsReflect.size() != 0)
    viewLaser1();

  //--------------------------------------------------------------------------
  if (m_msgRawdata2.bIsReflect.size() != 0)
    viewLaser2();

  //--------------------------------------------------------------------------
  drawFurniture();

  //--------------------------------------------------------------------------
  if (m_msgPlanPath.rps_route.size() != 0)
    drawPlanPath();

  //--------------------------------------------------------------------------
  if (m_msgRobotPath.rps_route.size() != 0)
    drawRobotPath();

  //--------------------------------------------------------------------------
  if (m_msgWagonPath.rps_route.size() != 0)
    drawWagonPath();

  //--------------------------------------------------------------------------
  if (m_msgRPS_MAP.rps_map_x.size() != 0)
    drawRPS_MAP();

  //--------------------------------------------------------------------------
  if (m_iViewSmartpal == MODE_SMARTPAL_ON)
  {
    if (m_bSmartpalTrajectory == true)
    {
      ros::Time tEnd = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
      ros::Time tStart = m_tSmartpalTrajectoryStartTime;

      drawSmartpalTrajectory(tStart, tEnd);
    }

    if (m_msgSmartpal.msgTMSInfo.size() != 0)
      if (m_msgSmartpal.msgTMSInfo[0].iState == STATE_EXIST)
        drawSmartpal(m_msgSmartpal.msgTMSInfo[0].fX, m_msgSmartpal.msgTMSInfo[0].fY,
                     m_msgSmartpal.msgTMSInfo[0].fTheta * DEG2RAD);
  }

  //--------------------------------------------------------------------------
  if (m_iViewRoomba == MODE_ROOMBA_ON)
  {
    if (m_bRoombaTrajectory == true)
    {
      ros::Time tEnd = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
      ros::Time tStart = m_tRoombaTrajectoryStartTime;

      drawRoombaTrajectory(tStart, tEnd);
    }

    if (m_msgRoomba.msgTMSInfo.size() != 0)
      if (m_msgRoomba.msgTMSInfo[0].iState == STATE_EXIST)
        drawRoomba(m_msgRoomba.msgTMSInfo[0].fX, m_msgRoomba.msgTMSInfo[0].fY,
                   m_msgRoomba.msgTMSInfo[0].fTheta * DEG2RAD);
  }

  //--------------------------------------------------------------------------
  if (m_msgChair.msgTMSInfo.size() != 0)
    if (m_msgChair.msgTMSInfo[0].iState == STATE_EXIST)
      drawChair();

  //--------------------------------------------------------------------------
  if (m_msgWagon.msgTMSInfo.size() != 0)
    if (m_msgWagon.msgTMSInfo[0].iState == STATE_EXIST)
      drawWagon();

  //--------------------------------------------------------------------------
  drawPersonTrajectory();

  if (m_msgIssWagon.msgTMSInfo.size() != 0)
    drawIssWagon();

  if (m_msgIssWheelChair.msgTMSInfo.size() != 0)
    drawIssWheelChair();

  if (m_msgIcsObjectData.msgTMSInfo.size() != 0)
    drawIcsObject();

  //--------------------------------------------------------------------------
  QPainter CPainter(&m_stImage);
  CPainter.setPen(QPen(Qt::black, 1));
  // CPainter.drawText(10, 10, "Mouse Point : "+QString::number(m_dMouse_point_x) + ", "+
  // QString::number(m_dMouse_point_y));

  POINT stLaserTemp;
  stLaserTemp.dX = 8000 * cos(m_iDataViewRowIndex * 0.25 * DEG2RAD) + m_dLrf_set_x;
  stLaserTemp.dY = 8000 * sin(m_iDataViewRowIndex * 0.25 * DEG2RAD) + m_dLrf_set_y;

  CPainter.setPen(QPen(Qt::black, 1));
  QLineF line(MM2Px(m_dLrf_set_x), MM2Py(m_dLrf_set_y), MM2Px(stLaserTemp.dX), MM2Py(stLaserTemp.dY));
  CPainter.drawLine(line);

  //--------------------------------------------------------------------------
  // View Occlusion
  if (m_iViewOcclusion == MODE_OCCLUSION_POINT)
  {
    for (unsigned int i = 0; i < m_msgClass.fOcclusionX.size(); i++)
    {
      int iX = MM2Px(m_msgClass.fOcclusionX[i]);
      int iY = MM2Py(m_msgClass.fOcclusionY[i]);

      CPainter.setPen(QPen(Qt::black, 5));
      CPainter.drawPoint(iX, iY);
    }
  }
  else if (m_iViewOcclusion == MODE_OCCLUSION_FACE)
  {
    for (unsigned int i = 0; i < m_msgClass.fOcclusionX.size(); i++)
    {
      int iX = MM2Px(m_msgClass.fOcclusionX[i] - 50);
      int iY = MM2Py(m_msgClass.fOcclusionY[i] + 50);

      CPainter.setPen(Qt::black);
      CPainter.drawRect(iX, iY, 14 * m_viewer_scale * 6.6, 14 * m_viewer_scale * 6.6);  // m_simulation_scale*6.6 = 1
    }
  }

  //--------------------------------------------------------------------------
  // View Cluster
  if (m_iViewCluster == MODE_CLUSTER_ON)
  {
    unsigned int iLength = m_msgCluster.iID.size();
    if (iLength != 0)
    {
      for (unsigned int i = 0; i < iLength; i++)
      {
        unsigned int iLengthx = m_msgCluster.LrfData[i].fX2.size();

        for (unsigned int j = 0; j < iLengthx; j++)
        {
          if (m_msgCluster.LrfData[i].bIsReflect[j] == true)
            CPainter.setPen(QPen(Qt::blue, iPointSize));
          else
            CPainter.setPen(QPen(Qt::red, iPointSize));

          CPainter.drawPoint(MM2Px(m_msgCluster.LrfData[i].fX2[j]), MM2Py(m_msgCluster.LrfData[i].fY2[j]));
        }

        CPainter.setPen(QPen(Qt::black, iPointSize));
        CPainter.drawPoint(MM2Px(m_msgCluster.fCenterX[i]), MM2Py(m_msgCluster.fCenterY[i]));

        if (m_bTagViewCluster == true)
        {
          CPainter.setPen(QPen(Qt::black, iPointSize));
          CPainter.drawText(MM2Px(m_msgCluster.fCenterX[i]), MM2Py(m_msgCluster.fCenterY[i] + 50),
                            "cID = " + QString::number(m_msgCluster.iID[i]));
        }
      }
    }
  }

  //--------------------------------------------------------------------------
  // View Class
  if (m_iViewClass == MODE_CLASS_ON)
  {
    unsigned int iLength = m_msgClass.iID.size();
    if (iLength != 0)
    {
      int color_id;
      for (unsigned int i = 0; i < iLength; i++)
      {
        if (m_bViewTwoColors)
        {
          if (m_msgClass.fAvgIntrinsicIntensity[i] >= m_iIntensityThreshold)
            color_id = 2;  // green
          else
            color_id = 3;  // blue
        }
        else
        {
          if (m_msgClass.iID[i] < 20)
            color_id = m_msgClass.iID[i] + 1;
          else
            color_id = (m_msgClass.iID[i] + 1) % 10;
        }

        unsigned int iLengthx = m_msgClass.LrfData[i].fX2.size();
        for (unsigned int j = 0; j < iLengthx; j++)
        {
          CPainter.setPen(QPen(QColor(qColorID[color_id]), iPointSize * 2));  // size: 2â†’3 for ICRA, SI, RS
          CPainter.drawPoint(MM2Px(m_msgClass.LrfData[i].fX2[j]), MM2Py(m_msgClass.LrfData[i].fY2[j]));
        }

        CPainter.setPen(QPen(Qt::black, iPointSize));
        CPainter.drawPoint(MM2Px(m_msgClass.fCenterX[i]), MM2Py(m_msgClass.fCenterY[i]));

        if (m_bTagViewClass == true)
        {
          CPainter.setPen(QPen(Qt::black, iPointSize));
          CPainter.drawText(MM2Px(m_msgClass.fCenterX[i]), MM2Py(m_msgClass.fCenterY[i] + 100),
                            "ID = " + QString::number(m_msgClass.iID[i]));
        }
      }
    }
  }

  if (m_bTagViewIntensity)
  {
    unsigned int iLength = m_msgClass.iID.size();
    if (iLength != 0)
    {
      for (unsigned int i = 0; i < iLength; i++)
      {
        CPainter.setPen(QPen(Qt::black, iPointSize));
        CPainter.drawText(MM2Px(m_msgClass.fCenterX[i]), MM2Py(m_msgClass.fCenterY[i] + 150),
                          QString::number(m_msgClass.fAvgIntrinsicIntensity[i]));
      }
    }
  }

  //--------------------------------------------------------------------------
  // View Unknown Class
  if (m_iViewUnknownClass == MODE_UNKNOWN_CLASS_ON)
  {
    unsigned int iLength = m_msgUnknownClass.iID.size();
    if (iLength != 0)
    {
      int color_id;
      for (unsigned int i = 0; i < iLength; i++)
      {
        if (m_bViewUnknownTwoColors)
        {
          if (m_msgUnknownClass.fAvgIntrinsicIntensity[i] >= m_iUnknownIntensityThreshold)
            color_id = 2;  // green
          else
            color_id = 3;  // blue
        }
        else
        {
          if (m_msgUnknownClass.iID[i] < 20)
            color_id = m_msgUnknownClass.iID[i] + 1;
          else
            color_id = (m_msgUnknownClass.iID[i] + 1) % 10;
        }

        unsigned int iLengthx = m_msgUnknownClass.LrfData[i].fX2.size();
        for (unsigned int j = 0; j < iLengthx; j++)
        {
          CPainter.setPen(QPen(QColor(qColorID[color_id]), iPointSize * 2));  // size: 2â†’3 for ICRA, SI, RS
          CPainter.drawPoint(MM2Px(m_msgUnknownClass.LrfData[i].fX2[j]), MM2Py(m_msgUnknownClass.LrfData[i].fY2[j]));
        }

        CPainter.setPen(QPen(Qt::black, iPointSize));
        CPainter.drawPoint(MM2Px(m_msgUnknownClass.fCenterX[i]), MM2Py(m_msgUnknownClass.fCenterY[i]));

        if (m_bTagViewUnknownClass == true)
        {
          CPainter.setPen(QPen(Qt::black, iPointSize));
          CPainter.drawText(MM2Px(m_msgUnknownClass.fCenterX[i]), MM2Py(m_msgUnknownClass.fCenterY[i] + 100),
                            "ID = " + QString::number(m_msgUnknownClass.iID[i]));
        }
      }
    }
  }

  /*
  if(m_bTagViewIntensity)
  {
      unsigned int iLength = m_msgUnknownClass.iID.size();
      if(iLength != 0)
      {
          for(unsigned int i=0; i < iLength; i++)
          {
              CPainter.setPen(QPen(Qt::black, iPointSize));
              CPainter.drawText(MM2Px(m_msgUnknownClass.fCenterX[i]), MM2Py(m_msgUnknownClass.fCenterY[i]+150),
  QString::number(m_msgUnknownClass.fAvgIntrinsicIntensity[i]));
          }
      }
  }
  */

  //--------------------------------------------------------------------------
  update();
}

//------------------------------------------------------------------------------
void Viewer::init_lrf(void)
{
  m_dLrf_scan_max_distance = 10000;  // mm

  m_dLrf_set_x = 2000;               // mm
  m_dLrf_set_y = 0;                  // mm
  m_dLrf_set_theta = DEG2RAD * 90.;  // radian

  m_dLrf_set_x1 = 5500;              // mm
  m_dLrf_set_y1 = 500;               // mm
  m_dLrf_set_theta1 = DEG2RAD * 90;  // radian

  m_dLrf_set_x2 = 6000;                 // mm
  m_dLrf_set_y2 = 4000;                 // mm
  m_dLrf_set_theta2 = DEG2RAD * (-90);  // radian

  m_dLrf_scan_max_range = DEG2RAD * 180.;
  m_dLrf_scan_max_count = ANGULAR_RESOLUTION;

  m_dWindow_size_x = width();
  m_dWindow_size_y = height();

  m_stImage.fill(qRgb(255, 255, 255));
}

//------------------------------------------------------------------------------
int Viewer::MM2P(double f_dP)
{
  return (int)(m_viewer_scale * f_dP);
}
int Viewer::MM2Px(double f_dX)
{
  return (int)(m_viewer_scale * (f_dX + m_dOffset_x));
}
int Viewer::MM2Py(double f_dY)
{
  return (int)(-m_viewer_scale * (f_dY + m_dOffset_y) + m_dWindow_size_y);
}
int Viewer::MM2Pl(double f_dD)
{
  return (int)(m_viewer_scale * f_dD);
}
double Viewer::P2MMx(int f_iX)
{
  return (f_iX) / m_viewer_scale - m_dOffset_x;
}
double Viewer::P2MMy(int f_iY)
{
  return (m_dWindow_size_y - f_iY) / m_viewer_scale - m_dOffset_y;
}
double Viewer::P2MMl(int f_iD)
{
  return (f_iD) / m_viewer_scale;
}

//------------------------------------------------------------------------------
void Viewer::clearImage()
{
  m_stImage.fill(qRgb(255, 255, 255));  // Qt::white
  update();
}

//------------------------------------------------------------------------------
void Viewer::mousePressEvent(QMouseEvent* event)
{
  setFocus();

  if (event->button() == Qt::RightButton)
  {
    m_stLast_point.setX(P2MMx(event->pos().x()));
    m_stLast_point.setY(P2MMy(event->pos().y()));
  }
}

//------------------------------------------------------------------------------
void Viewer::mouseMoveEvent(QMouseEvent* event)
{
  m_dMouse_point_x = P2MMx(event->pos().x());
  m_dMouse_point_y = P2MMy(event->pos().y());

  if (event->buttons() & Qt::RightButton)
  {
    int x, y = 0;
    x = m_dMouse_point_x - m_stLast_point.x();
    y = m_dMouse_point_y - m_stLast_point.y();

    m_dOffset_x += x;
    m_dOffset_y += y;
  }
}

//------------------------------------------------------------------------------
void Viewer::mouseReleaseEvent(QMouseEvent* event)
{
}

//------------------------------------------------------------------------------
void Viewer::resizeEvent(QResizeEvent* event)
{
  QImage newImage(event->size(), QImage::Format_RGB32);
  newImage.fill(qRgb(255, 255, 255));  // Qt::white
  QPainter CPainter(this);
  CPainter.drawImage(QPoint(0, 0), m_stImage);
  m_stImage = newImage;
}

//------------------------------------------------------------------------------
void Viewer::paintEvent(QPaintEvent* event)
{
  QPainter CPainter(this);
  QRect dirtyRect = event->rect();
  CPainter.drawImage(dirtyRect, m_stImage, dirtyRect);
}

//------------------------------------------------------------------------------
void Viewer::keyPressEvent(QKeyEvent* event)
{
}

//------------------------------------------------------------------------------
void Viewer::wheelEvent(QWheelEvent* event)
{
  int numDegrees = event->delta() / 8;
  int numSteps = numDegrees / 15;

  if (event->orientation() == Qt::Horizontal)
  {
  }
  else
  {
    if (m_iSelect_object == 0)
    {
      m_viewer_scale += numSteps * 0.01;
    }
  }
  event->accept();
}

//------------------------------------------------------------------------------
void Viewer::load_map()
{
  m_vstMap_2d.clear();

  LINE e0, e1, e2, e3;
  double MaxMapSizeX = 8000;
  double MaxMapSizeY = 4500;

  e0.dX0 = 0;
  e0.dY0 = 0;
  e0.dX1 = MaxMapSizeX;
  e0.dY1 = 0;

  e1.dX0 = MaxMapSizeX;
  e1.dY0 = 0;
  e1.dX1 = MaxMapSizeX;
  e1.dY1 = MaxMapSizeY;

  e2.dX0 = MaxMapSizeX;
  e2.dY0 = MaxMapSizeY;
  e2.dX1 = 0;
  e2.dY1 = MaxMapSizeY;

  e3.dX0 = 0;
  e3.dY0 = MaxMapSizeY;
  e3.dX1 = 0;
  e3.dY1 = 0;

  m_vstMap_2d.push_back(e0);
  m_vstMap_2d.push_back(e1);
  m_vstMap_2d.push_back(e2);
  m_vstMap_2d.push_back(e3);
}

//------------------------------------------------------------------------------
void Viewer::load_grid()
{
  m_vstGrid_2d.clear();

  LINE point[23];
  double maxMapSizeX = 8000;
  double maxMapSizeY = 4500;
  double interval = 500;

  for (int n = 0; n < 15; n++)
  {
    point[n].dX0 = interval * (n + 1);
    point[n].dY0 = 0;
    point[n].dX1 = interval * (n + 1);
    point[n].dY1 = maxMapSizeY;
  }
  for (int n = 0; n < 8; n++)
  {
    point[n + 15].dX0 = 0;
    point[n + 15].dY0 = interval * (n + 1);
    point[n + 15].dX1 = maxMapSizeX;
    point[n + 15].dY1 = interval * (n + 1);
  }

  for (int n = 0; n < 23; n++)
    m_vstGrid_2d.push_back(point[n]);
}

//------------------------------------------------------------------------------
void Viewer::drawMap()
{
  for (unsigned int i = 0; i < m_vstMap_2d.size(); ++i)
  {
    int a, b, c, d;
    QPainter CPainter(&m_stImage);
    CPainter.setPen(QPen(Qt::blue, 1));
    a = MM2Px(m_vstMap_2d[i].dX0);
    b = MM2Py(m_vstMap_2d[i].dY0);
    c = MM2Px(m_vstMap_2d[i].dX1);
    d = MM2Py(m_vstMap_2d[i].dY1);
    QLineF line(a, b, c, d);
    CPainter.drawLine(line);
    update();
  }
}

//------------------------------------------------------------------------------
void Viewer::drawGridSquare()
{
  for (unsigned int i = 0; i < m_vstGrid_2d.size(); ++i)
  {
    int a, b, c, d;
    QPainter CPainter(&m_stImage);
    CPainter.setPen(QPen(Qt::lightGray, 1));
    a = MM2Px(m_vstGrid_2d[i].dX0);
    b = MM2Py(m_vstGrid_2d[i].dY0);
    c = MM2Px(m_vstGrid_2d[i].dX1);
    d = MM2Py(m_vstGrid_2d[i].dY1);
    QLineF line(a, b, c, d);
    CPainter.drawLine(line);
    update();
  }
}

//------------------------------------------------------------------------------
void Viewer::drawGridCircle()
{
  QPainter CPainter(&m_stImage);
  CPainter.setPen(QPen(Qt::lightGray, 1));

  QRectF rectangle1(MM2Px(1500), MM2Py(500), MM2P(1000), MM2P(1000));
  QRectF rectangle2(MM2Px(1000), MM2Py(1000), MM2P(2000), MM2P(2000));
  QRectF rectangle3(MM2Px(500), MM2Py(1500), MM2P(3000), MM2P(3000));
  QRectF rectangle4(MM2Px(0), MM2Py(2000), MM2P(4000), MM2P(4000));
  QRectF rectangle5(MM2Px(-500), MM2Py(2500), MM2P(5000), MM2P(5000));
  QRectF rectangle6(MM2Px(-1000), MM2Py(3000), MM2P(6000), MM2P(6000));
  QRectF rectangle7(MM2Px(-1500), MM2Py(3500), MM2P(7000), MM2P(7000));
  QRectF rectangle8(MM2Px(-2000), MM2Py(4000), MM2P(8000), MM2P(8000));
  QRectF rectangle9(MM2Px(-2500), MM2Py(4500), MM2P(9000), MM2P(9000));
  /*
      QRectF rectangle1(MM2Px(2500), MM2Py(500),  MM2P(1000), MM2P(1000));
      QRectF rectangle2(MM2Px(2000), MM2Py(1000), MM2P(2000), MM2P(2000));
      QRectF rectangle3(MM2Px(1500), MM2Py(1500), MM2P(3000), MM2P(3000));
      QRectF rectangle4(MM2Px(1000), MM2Py(2000), MM2P(4000), MM2P(4000));
      QRectF rectangle5(MM2Px(500),  MM2Py(2500), MM2P(5000), MM2P(5000));
      QRectF rectangle6(MM2Px(0),    MM2Py(3000), MM2P(6000), MM2P(6000));
      QRectF rectangle7(MM2Px(-500), MM2Py(3500), MM2P(7000), MM2P(7000));
      QRectF rectangle8(MM2Px(-1000),MM2Py(4000), MM2P(8000), MM2P(8000));
      QRectF rectangle9(MM2Px(-1500),MM2Py(4500), MM2P(9000), MM2P(9000));
  */

  int startAngle = 0 * 16;   // 0 degree
  int spanAngle = 180 * 16;  // 180 degree

  CPainter.drawArc(rectangle1, startAngle, spanAngle);
  CPainter.drawArc(rectangle2, startAngle, spanAngle);
  CPainter.drawArc(rectangle3, startAngle, spanAngle);
  CPainter.drawArc(rectangle4, startAngle, spanAngle);
  CPainter.drawArc(rectangle5, startAngle, spanAngle);
  CPainter.drawArc(rectangle6, startAngle, spanAngle);
  CPainter.drawArc(rectangle7, startAngle, spanAngle);
  CPainter.drawArc(rectangle8, startAngle, spanAngle);
  CPainter.drawArc(rectangle9, startAngle, spanAngle);

  update();
}

//------------------------------------------------------------------------------
void Viewer::drawFurniture(void)
{
  int iX, iY;
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 20);

  if (m_iViewFurniture == MODE_FURNITURE_ON)
  {
    // draw bed
    iX = MM2Px(25);
    iY = MM2Py(2140);
    CPainter.setPen(QPen(Qt::darkGreen, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 950, m_viewer_scale * 2080);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "BED");

    // draw chair
    iX = MM2Px(550);
    iY = MM2Py(4100);
    CPainter.setPen(QPen(Qt::darkRed, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 410, m_viewer_scale * 470);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "CHAIR");

    // draw desk
    iX = MM2Px(0);
    iY = MM2Py(4500);
    CPainter.setPen(QPen(Qt::darkMagenta, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 1400, m_viewer_scale * 580);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "DESK");

    // draw intelligent cabinet
    iX = MM2Px(4100);
    iY = MM2Py(2000);
    CPainter.setPen(QPen(Qt::darkYellow, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 305, m_viewer_scale * 460);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "i-CABINET");

    // graw big_shelf
    iX = MM2Px(1600);
    iY = MM2Py(4500);
    CPainter.setPen(QPen(Qt::darkYellow, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 803, m_viewer_scale * 293);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "SHELF");

    // draw chair
    iX = MM2Px(3300);
    iY = MM2Py(2900);
    CPainter.setPen(QPen(Qt::darkRed, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 410, m_viewer_scale * 470);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "CHAIR");

    // draw chair
    iX = MM2Px(3300);
    iY = MM2Py(2000);
    CPainter.setPen(QPen(Qt::darkRed, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 410, m_viewer_scale * 470);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "CHAIR");

    // draw table
    iX = MM2Px(3100);
    iY = MM2Py(2600);
    CPainter.setPen(QPen(Qt::darkMagenta, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 800, m_viewer_scale * 800);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "TABLE");

    // draw cabinet
    //~ iX = MM2Px(3920);
    //~ iY = MM2Py(1070);
    //~ CPainter.setPen(QPen(Qt::darkYellow, iPointSize));
    //~ CPainter.drawRect(iX, iY, m_viewer_scale*290, m_viewer_scale*420);
    //~ if(m_bTagViewFurniture == true)
    //~ CPainter.drawText(MM2Px(4240), MM2Py(1000), "CABINET");

    // draw partition
    iX = MM2Px(4000);
    iY = MM2Py(3000);
    CPainter.setPen(QPen(Qt::darkYellow, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 10, m_viewer_scale * 1600);
    CPainter.setPen(QPen(Qt::darkYellow, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 900, m_viewer_scale * 10);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "PARTITION");

    // draw tv table
    iX = MM2Px(6200);
    iY = MM2Py(405);
    CPainter.setPen(QPen(Qt::darkMagenta, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 1200, m_viewer_scale * 400);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY + 25, "TVTABLE");

    // draw sofa table
    iX = MM2Px(6400);
    iY = MM2Py(3300);
    CPainter.setPen(QPen(Qt::darkGreen, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 1050, m_viewer_scale * 600);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "SOFA-TABLE");

    // draw mini sofa
    iX = MM2Px(7450);
    iY = MM2Py(3300);
    CPainter.setPen(QPen(Qt::darkRed, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 525, m_viewer_scale * 525);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "MINISOFA");

    // draw sofa
    iX = MM2Px(6400);
    iY = MM2Py(4400);
    CPainter.setPen(QPen(Qt::darkRed, iPointSize));
    CPainter.drawRect(iX, iY, m_viewer_scale * 1492, m_viewer_scale * 766);
    if (m_bTagViewFurniture == true)
      CPainter.drawText(iX + 5, iY - 5, "SOFA");
  }
}

//------------------------------------------------------------------------------
void Viewer::drawPlanPath(void)
{
  float fNowX, fNowY, fNextX, fNextY, f_dX, f_dY, f_dTheta;
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 10);
  int iLineSize = (int)(m_viewer_scale * 5);

  if (m_bPlanPathView == MODE_PLAN_PATH_ON)
  {
    if (m_iViewPath == MODE_PATH_ON)
    {
      for (unsigned int i = 0; i < m_msgPlanPath.rps_route.size() - 1; i++)
      {
        fNowX = m_msgPlanPath.rps_route[i].x;
        fNowY = m_msgPlanPath.rps_route[i].y;
        fNextX = m_msgPlanPath.rps_route[i + 1].x;
        fNextY = m_msgPlanPath.rps_route[i + 1].y;

        // trajectory line
        CPainter.setPen(QPen(Qt::darkRed, (iPointSize / 2) * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
        QLineF qtLine(MM2Px(fNowX), MM2Py(fNowY), MM2Px(fNextX), MM2Py(fNextY));
        CPainter.drawLine(qtLine);

        // trajectory point
        CPainter.setPen(QPen(Qt::red, iPointSize * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
        CPainter.drawPoint(MM2Px(fNowX), MM2Py(fNowY));
      }
    }
    CPainter.setPen(QPen(Qt::black, iPointSize * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
    CPainter.drawPoint(MM2Px(m_msgPlanPath.rps_route[m_msgPlanPath.rps_route.size() - 1].x),
                       MM2Py(m_msgPlanPath.rps_route[m_msgPlanPath.rps_route.size() - 1].y));
  }
}

//------------------------------------------------------------------------------
void Viewer::drawRobotPath(void)
{
  float fNowX, fNowY, fNextX, fNextY, f_dX, f_dY, f_dTheta;
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 10);
  int iLineSize = (int)(m_viewer_scale * 5);

  if ((m_bRobotPathView == MODE_ROBOT_PATH_ON) && (m_iViewSmartpal == MODE_SMARTPAL_ON))
  {
    for (unsigned int i = 0; i < m_msgRobotPath.rps_route.size(); i++)
    {
      bool even;
      if (i % 2 == 0)
        even = true;
      else
        even = false;

      // set point
      f_dX = m_msgRobotPath.rps_route[i].x;
      f_dY = m_msgRobotPath.rps_route[i].y;
      f_dTheta = m_msgRobotPath.rps_route[i].th;

      int robot_id = m_msgRobotPath.robot_id;

      int num;
      if (robot_id == 2002)
        num = 28;
      else if (robot_id == 2006)
        num = 9;
      else
        num = 24;
      QPoint CRobot_points[num];

      switch (robot_id)
      {
        case 2002:  // smartpal5_1
        {
          QPoint CSmartpal4_points[28] =  // smartpal4
              {QPoint(356, 0),     QPoint(356, -79),   QPoint(309, -81),   QPoint(277, -196), QPoint(251, -254),
               QPoint(224, -300),  QPoint(170, -304),  QPoint(81, -304),   QPoint(0, -304),   QPoint(-74, -301),
               QPoint(-142, -287), QPoint(-199, -256), QPoint(-260, -198), QPoint(-315, -96), QPoint(-331, 0),
               QPoint(-315, 96),   QPoint(-260, 198),  QPoint(-199, 256),  QPoint(-142, 287), QPoint(-74, 301),
               QPoint(0, 304),     QPoint(81, 304),    QPoint(170, 304),   QPoint(224, 300),  QPoint(251, 254),
               QPoint(277, 196),   QPoint(309, 81),    QPoint(356, 79)};
          for (int i = 0; i < num; i++)
            CRobot_points[i] = CSmartpal4_points[i];
          break;
        }
        case 2005:  // kobuki
        {
          QPoint Ckobuki_points[24] =  // kobuki
              {QPoint(150, 0),    QPoint(145, -39),  QPoint(130, -75),  QPoint(106, -106), QPoint(75, -130),
               QPoint(39, -145),  QPoint(0, -150),   QPoint(-39, -145), QPoint(-75, -130), QPoint(-106, -106),
               QPoint(-130, -75), QPoint(-145, -39), QPoint(-150, 0),   QPoint(-145, 39),  QPoint(-130, 75),
               QPoint(-106, 106), QPoint(-75, 130),  QPoint(-39, 145),  QPoint(0, 150),    QPoint(39, 145),
               QPoint(75, 130),   QPoint(106, 106),  QPoint(130, 75),   QPoint(145, 39)};
          for (int i = 0; i < num; i++)
            CRobot_points[i] = Ckobuki_points[i];
          break;
        }
        case 2006:  // kxp
        {
          QPoint Ckxp_points[9] =  // kxp
              {QPoint(250, 0),    QPoint(250, -245), QPoint(0, -245),  QPoint(-250, -245), QPoint(-250, 0),
               QPoint(-250, 245), QPoint(0, 245),    QPoint(250, 245), QPoint(250, 0)};
          for (int i = 0; i < num; i++)
            CRobot_points[i] = Ckxp_points[i];
          break;
        }
        default:
        {
          QPoint CSmartpal5_points[24] =  // smartpal5
              {QPoint(237, 0),     QPoint(229, -72),  QPoint(205, -139),  QPoint(168, -196),  QPoint(119, -240),
               QPoint(61, -268),   QPoint(0, -278),   QPoint(-103, -268), QPoint(-192, -240), QPoint(-268, -196),
               QPoint(-327, -139), QPoint(-363, -72), QPoint(-376, 0),    QPoint(-363, 72),   QPoint(-327, 139),
               QPoint(-268, 196),  QPoint(-192, 240), QPoint(-103, 268),  QPoint(0, 278),     QPoint(61, 268),
               QPoint(119, 240),   QPoint(168, 196),  QPoint(205, 139),   QPoint(229, 72)};
          for (int i = 0; i < num; i++)
            CRobot_points[i] = CSmartpal5_points[i];
        }
      }
      int iRobot_max_point_index = sizeof(CRobot_points) / sizeof(CRobot_points[0]);

      for (int j = 0; j < iRobot_max_point_index; j++)
      {
        CRobot_points[j].setX(CRobot_points[j].x() + f_dX);
        CRobot_points[j].setY(CRobot_points[j].y() + f_dY);
      }

      for (int j = 0; j < iRobot_max_point_index; j++)
      {
        int iRotaionX = (CRobot_points[j].x() - f_dX) * cos(f_dTheta) - (CRobot_points[j].y() - f_dY) * sin(f_dTheta);
        int iRotaionY = (CRobot_points[j].x() - f_dX) * sin(f_dTheta) + (CRobot_points[j].y() - f_dY) * cos(f_dTheta);

        CRobot_points[j].setX(iRotaionX + f_dX);
        CRobot_points[j].setY(iRotaionY + f_dY);
      }

      for (int j = 0; j < iRobot_max_point_index; j++)
      {
        CRobot_points[j].setX(MM2Px(CRobot_points[j].x()));
        CRobot_points[j].setY(MM2Py(CRobot_points[j].y()));
      }

      CPainter.setPen(QPen(Qt::gray, iPointSize / 2));
      CPainter.drawPolygon(CRobot_points, iRobot_max_point_index);
      // kobuki角度表示
      if (robot_id == 2005)
      {
        if (even == true)
        {
          CPainter.setPen(QPen(Qt::green, iPointSize * 2));
          int iRotaionX = (150.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
          int iRotaionY = (150.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);
          CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
        }
        else
        {
          CPainter.setPen(QPen(Qt::red, iPointSize * 4));
          int iRotaionX = (150.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
          int iRotaionY = (150.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);
          CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
        }
      }
      else if (robot_id == 2006)
      {
        if (even == true)
        {
          CPainter.setPen(QPen(Qt::green, iPointSize * 2));

          int iRotaionX = (250.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
          int iRotaionY = (250.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);
          CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
        }
        else
        {
          CPainter.setPen(QPen(Qt::red, iPointSize * 4));

          int iRotaionX = (250.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
          int iRotaionY = (250.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);
          CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
        }
      }
    }
  }

  if ((m_iViewPath == MODE_PATH_ON) && (m_bRobotPathView == MODE_ROBOT_PATH_ON))
  {
    for (unsigned int i = 0; i < m_msgRobotPath.rps_route.size() - 1; i++)
    {
      // set point
      fNowX = m_msgRobotPath.rps_route[i].x;
      fNowY = m_msgRobotPath.rps_route[i].y;
      fNextX = m_msgRobotPath.rps_route[i + 1].x;
      fNextY = m_msgRobotPath.rps_route[i + 1].y;

      // trajectory line
      CPainter.setPen(QPen(Qt::darkGray, (iPointSize / 2) * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
      QLineF qtLine(MM2Px(fNowX), MM2Py(fNowY), MM2Px(fNextX), MM2Py(fNextY));
      CPainter.drawLine(qtLine);

      // trajectory point
      CPainter.setPen(QPen(Qt::black, iPointSize * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
      CPainter.drawPoint(MM2Px(fNowX), MM2Py(fNowY));
    }
    CPainter.setPen(QPen(Qt::black, iPointSize * (int)(450 / (m_msgRPS_MAP.rps_map_x.size() - 1))));
    CPainter.drawPoint(MM2Px(m_msgRobotPath.rps_route[m_msgRobotPath.rps_route.size() - 1].x),
                       MM2Py(m_msgRobotPath.rps_route[m_msgRobotPath.rps_route.size() - 1].y));
  }
}

//------------------------------------------------------------------------------
void Viewer::drawWagonPath(void)
{
  float f_dX, f_dY, f_dTheta;
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  if (m_bWagonPathView == MODE_WAGON_PATH_ON)
  {
    for (unsigned int i = 0; i < m_msgWagonPath.rps_route.size(); i++)
    {
      // set point
      f_dX = m_msgWagonPath.rps_route[i].x;
      f_dY = m_msgWagonPath.rps_route[i].y;
      f_dTheta = m_msgWagonPath.rps_route[i].th;

      QPoint CWagon_points[4] = {QPoint(275, 138), QPoint(-275, 138), QPoint(-275, -138), QPoint(275, -138)};

      int iWagon_max_point_index = sizeof(CWagon_points) / sizeof(CWagon_points[0]);

      for (int j = 0; j < iWagon_max_point_index; j++)
      {
        CWagon_points[j].setX(CWagon_points[j].x() + f_dX);
        CWagon_points[j].setY(CWagon_points[j].y() + f_dY);
      }

      for (int j = 0; j < iWagon_max_point_index; j++)
      {
        int iRotaionX = (CWagon_points[j].x() - f_dX) * cos(f_dTheta) - (CWagon_points[j].y() - f_dY) * sin(f_dTheta);
        int iRotaionY = (CWagon_points[j].x() - f_dX) * sin(f_dTheta) + (CWagon_points[j].y() - f_dY) * cos(f_dTheta);

        CWagon_points[j].setX(iRotaionX + f_dX);
        CWagon_points[j].setY(iRotaionY + f_dY);
      }

      for (int j = 0; j < iWagon_max_point_index; j++)
      {
        CWagon_points[j].setX(MM2Px(CWagon_points[j].x()));
        CWagon_points[j].setY(MM2Py(CWagon_points[j].y()));
      }

      CPainter.setPen(QPen(Qt::green, iPointSize / 2));
      CPainter.drawPolygon(CWagon_points, iWagon_max_point_index);
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawRPS_MAP(void)
{
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 10);

  int MaxMapSizeX = 800;
  int MaxMapSizeY = 450;

  if (m_iViewRPS_MAP == MODE_RPS_MAP_ON)
  {
    for (unsigned int x = 0; x < m_msgRPS_MAP.rps_map_x.size(); x++)
    {
      for (unsigned int y = 0; y < m_msgRPS_MAP.rps_map_x[x].rps_map_y.size(); y++)
      {
        if (m_bCollisionAreaViewRPS_MAP == true)
        {
          if (m_msgRPS_MAP.rps_map_x[x].rps_map_y[y].robot_collision)
          {
            CPainter.setPen(QPen(Qt::red, iPointSize * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))));
            CPainter.drawPoint(MM2Px(x * 10 * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))),
                               MM2Py(y * 10 * (int)(MaxMapSizeY / (m_msgRPS_MAP.rps_map_x[x].rps_map_y.size() - 1))));
          }
          //~ if(m_msgRPS_MAP.rps_map_x[x].rps_map_y[y].wagon_collision){
          //~ CPainter.setPen(QPen(Qt::darkRed, iPointSize*(int)(MaxMapSizeX/m_msgRPS_MAP.rps_map_x.size())));
          //~ CPainter.drawPoint(MM2Px(x*10), MM2Py(y*10));
          //~ }
        }
        if (m_msgRPS_MAP.rps_map_x[x].rps_map_y[y].object)
        {
          CPainter.setPen(QPen(Qt::black, iPointSize * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))));
          CPainter.drawPoint(MM2Px(x * 10 * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))),
                             MM2Py(y * 10 * (int)(MaxMapSizeY / (m_msgRPS_MAP.rps_map_x[x].rps_map_y.size() - 1))));
        }
        if (m_bvoronoiLineViewRPS_MAP == true)
        {
          if (m_msgRPS_MAP.rps_map_x[x].rps_map_y[y].voronoi)
          {
            CPainter.setPen(QPen(Qt::blue, iPointSize * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))));
            CPainter.drawPoint(MM2Px(x * 10 * (int)(MaxMapSizeX / (m_msgRPS_MAP.rps_map_x.size() - 1))),
                               MM2Py(y * 10 * (int)(MaxMapSizeY / (m_msgRPS_MAP.rps_map_x[x].rps_map_y.size() - 1))));
          }
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawSmartpal(float f_dX, float f_dY, float f_dTheta)
{
  QPainter CPainter(&m_stImage);

  QPoint CSmartpal_points[28] = {QPoint(356, 0),     QPoint(356, -79),  QPoint(309, -81),   QPoint(277, -196),
                                 QPoint(251, -254),  QPoint(224, -300), QPoint(170, -304),  QPoint(81, -304),
                                 QPoint(0, -304),    QPoint(-74, -301), QPoint(-142, -287), QPoint(-199, -256),
                                 QPoint(-260, -198), QPoint(-315, -96), QPoint(-331, 0),    QPoint(-315, 96),
                                 QPoint(-260, 198),  QPoint(-199, 256), QPoint(-142, 287),  QPoint(-74, 301),
                                 QPoint(0, 304),     QPoint(81, 304),   QPoint(170, 304),   QPoint(224, 300),
                                 QPoint(251, 254),   QPoint(277, 196),  QPoint(309, 81),    QPoint(356, 79)};

  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  if (m_iViewSmartpal == MODE_SMARTPAL_ON)
  {
    CPainter.setPen(QPen(Qt::darkBlue, iPointSize / 2));
    CPainter.setBrush(Qt::lightGray);
    //
    int iRobot_max_point_index = sizeof(CSmartpal_points) / sizeof(CSmartpal_points[0]);

    for (int i = 0; i < iRobot_max_point_index; i++)
    {
      CSmartpal_points[i].setX(CSmartpal_points[i].x() + f_dX);
      CSmartpal_points[i].setY(CSmartpal_points[i].y() + f_dY);
    }

    for (int i = 0; i < iRobot_max_point_index; i++)
    {
      int iRotaionX =
          (CSmartpal_points[i].x() - f_dX) * cos(f_dTheta) - (CSmartpal_points[i].y() - f_dY) * sin(f_dTheta);
      int iRotaionY =
          (CSmartpal_points[i].x() - f_dX) * sin(f_dTheta) + (CSmartpal_points[i].y() - f_dY) * cos(f_dTheta);

      CSmartpal_points[i].setX(iRotaionX + f_dX);
      CSmartpal_points[i].setY(iRotaionY + f_dY);
    }

    for (int i = 0; i < iRobot_max_point_index; i++)
    {
      CSmartpal_points[i].setX(MM2Px(CSmartpal_points[i].x()));
      CSmartpal_points[i].setY(MM2Py(CSmartpal_points[i].y()));
    }

    CPainter.drawPolygon(CSmartpal_points, iRobot_max_point_index);

    //
    for (unsigned int i = 0; i < m_msgSmartpal.fVirtualPointX.size(); i++)
    {
      CPainter.setPen(QPen(Qt::black, iPointSize));

      CPainter.drawPoint(MM2Px(m_msgSmartpal.fVirtualPointX[i]), MM2Py(m_msgSmartpal.fVirtualPointY[i]));
    }

    QPoint qCenter(MM2Px(f_dX), MM2Py(f_dY));

    CPainter.setPen(QPen(Qt::darkBlue, iPointSize / 2));

    int iRotaionX = (200.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
    int iRotaionY = (200.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);

    QString qsX;
    QString qsY;
    QString qsTheta;

    CPainter.setPen(QPen(Qt::black, iPointSize));
    CPainter.drawPoint(MM2Px(f_dX), MM2Py(f_dY));

    if (m_bTagViewSmartpal == true)
    {
      CPainter.setPen(QPen(Qt::black, iLineSize * 8));
      CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
      CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 630), "X = " + qsX.setNum(f_dX, 'f', 0) + "[mm]");
      CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 530), "Y = " + qsY.setNum(f_dY, 'f', 0) + "[mm]");
      CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 430),
                        "Theta = " + qsTheta.setNum(f_dTheta * RAD2DEG, 'f', 0) + "[degree]");

      CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
      CPainter.setBrush(Qt::NoBrush);
      QPoint qCenter(MM2Px(f_dX), MM2Py(f_dY));
      CPainter.drawEllipse(qCenter, MM2P(SMARTPAL_R), MM2P(SMARTPAL_R));
    }
  }
}

//------------------------------------------------------------------------------
bool Viewer::drawSmartpalTrajectory(ros::Time tStart, ros::Time tEnd)
{
  QPainter CPainter(&m_stImage);

  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  m_srvSmartpalTrajectory.request.iID = ID_SMARTPAL;
  m_srvSmartpalTrajectory.request.tStartTime = tStart;
  m_srvSmartpalTrajectory.request.tEndTime = tEnd;

  if (rosclientTmsdbRobotTrajectoryData.call(m_srvSmartpalTrajectory))
  {
    if (m_srvSmartpalTrajectory.response.msgTMSInfo.size() > 0)
      ROS_INFO("Success SRV Receive : %d", m_srvSmartpalTrajectory.response.msgTMSInfo[0].iID);
    else
      ROS_INFO("msgTMSInfo.size() = 0");
  }
  else
  {
    ROS_INFO("Failed to call service tmsdb_robot_trajectory_data");
    return false;
  }

  if (m_srvSmartpalTrajectory.response.msgTMSInfo.size() > 0)
  {
    for (unsigned int i = 0; i < m_srvSmartpalTrajectory.response.msgTMSInfo.size() - 1; i++)
    {
      float fNowX, fNowY, fNextX, fNextY;

      fNowX = m_srvSmartpalTrajectory.response.msgTMSInfo[i].fX;
      fNowY = m_srvSmartpalTrajectory.response.msgTMSInfo[i].fY;
      fNextX = m_srvSmartpalTrajectory.response.msgTMSInfo[i + 1].fX;
      fNextY = m_srvSmartpalTrajectory.response.msgTMSInfo[i + 1].fY;

      // trajectory line
      CPainter.setPen(QPen(Qt::darkGray, iPointSize));
      QLineF qtLine(MM2Px(fNowX), MM2Py(fNowY), MM2Px(fNextX), MM2Py(fNextY));
      CPainter.drawLine(qtLine);

      // trajectory point
      CPainter.setPen(QPen(Qt::black, iPointSize * 2));
      CPainter.drawPoint(MM2Px(fNowX), MM2Py(fNowY));

      // trajectory ellipse
      CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
      CPainter.setBrush(Qt::NoBrush);
      QPoint qCenter(MM2Px(fNowX), MM2Py(fNowY));
      CPainter.drawEllipse(qCenter, MM2P(100), MM2P(100));
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void Viewer::drawRoomba(float f_dX, float f_dY, float f_dTheta)
{
  QPainter CPainter(&m_stImage);

  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  QPoint qCenter(MM2Px(f_dX), MM2Py(f_dY));

  CPainter.setPen(QPen(Qt::darkBlue, iPointSize));
  CPainter.setBrush(Qt::green);
  CPainter.drawEllipse(qCenter, MM2P(170), MM2P(170));

  int iRotaionX = (200.0) * cos(f_dTheta) - (0.0) * sin(f_dTheta);
  int iRotaionY = (200.0) * sin(f_dTheta) + (0.0) * cos(f_dTheta);

  QString qsX;
  QString qsY;
  QString qsTheta;

  CPainter.setPen(QPen(Qt::black, iPointSize));
  CPainter.drawPoint(MM2Px(f_dX), MM2Py(f_dY));

  if (m_bTagViewRoomba == true)
  {
    CPainter.setPen(QPen(Qt::black, iLineSize * 8));
    CPainter.drawLine(MM2Px(f_dX), MM2Py(f_dY), MM2Px(iRotaionX + f_dX), MM2Py(iRotaionY + f_dY));
    CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 420), "X = " + qsX.setNum(f_dX, 'f', 0) + "[mm]");
    CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 320), "Y = " + qsY.setNum(f_dY, 'f', 0) + "[mm]");
    CPainter.drawText(MM2Px(f_dX - 200), MM2Py(f_dY + 220),
                      "Theta = " + qsTheta.setNum(f_dTheta * RAD2DEG, 'f', 1) + "[degree]");

    CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
    CPainter.setBrush(Qt::NoBrush);
    QPoint qCenter(MM2Px(f_dX), MM2Py(f_dY));
    CPainter.drawEllipse(qCenter, MM2P(ROOMBA_R), MM2P(ROOMBA_R));
  }
}

//------------------------------------------------------------------------------
bool Viewer::drawRoombaTrajectory(ros::Time tStart, ros::Time tEnd)
{
  QPainter CPainter(&m_stImage);

  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  m_srvRoombaTrajectory.request.iID = ID_ROOMBA;
  m_srvRoombaTrajectory.request.tStartTime = tStart;
  m_srvRoombaTrajectory.request.tEndTime = tEnd;

  if (rosclientTmsdbRobotTrajectoryData.call(m_srvRoombaTrajectory))
  {
    if (m_srvRoombaTrajectory.response.msgTMSInfo.size() > 0)
      ROS_INFO("Success SRV Receive : %d", m_srvRoombaTrajectory.response.msgTMSInfo[0].iID);
    else
      ROS_INFO("msgTMSInfo.size() = 0");
  }
  else
  {
    ROS_INFO("Failed to call service tmsdb_robot_trajectory_data");
    return false;
  }

  if (m_srvRoombaTrajectory.response.msgTMSInfo.size() > 0)
  {
    for (unsigned int i = 0; i < m_srvRoombaTrajectory.response.msgTMSInfo.size() - 1; i++)
    {
      float fNowX, fNowY, fNextX, fNextY;

      fNowX = m_srvRoombaTrajectory.response.msgTMSInfo[i].fX;
      fNowY = m_srvRoombaTrajectory.response.msgTMSInfo[i].fY;
      fNextX = m_srvRoombaTrajectory.response.msgTMSInfo[i + 1].fX;
      fNextY = m_srvRoombaTrajectory.response.msgTMSInfo[i + 1].fY;

      // trajectory line
      CPainter.setPen(QPen(Qt::darkBlue, iPointSize));
      QLineF qtLine(MM2Px(fNowX), MM2Py(fNowY), MM2Px(fNextX), MM2Py(fNextY));
      CPainter.drawLine(qtLine);

      // trajectory point
      CPainter.setPen(QPen(Qt::black, iPointSize * 2));
      CPainter.drawPoint(MM2Px(fNowX), MM2Py(fNowY));

      // trajectory ellipse
      CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
      CPainter.setBrush(Qt::NoBrush);
      QPoint qCenter(MM2Px(fNowX), MM2Py(fNowY));
      CPainter.drawEllipse(qCenter, MM2P(70), MM2P(70));
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void Viewer::drawWagon(void)
{
  QPainter CPainter(&m_stImage);

  int iLineSize = (int)(m_viewer_scale * 5);

  float fX = 0.0;
  float fY = 0.0;
  // float fTheta = 0.0;

  fX = m_msgWagon.msgTMSInfo[0].fX;
  fY = m_msgWagon.msgTMSInfo[0].fY;
  // fTheta = m_msgWagon.fDirection;

  QPoint CWagon_points[4] = {QPoint(270, 140), QPoint(-270, 140), QPoint(-270, -140), QPoint(270, -140)};

  if (m_iViewWagon == MODE_WAGON_ON)
  {
    for (int i = 0; i < m_msgWagon.iGroupsCount; i++)
    {
      CPainter.setPen(QPen(Qt::red, 10));
      CPainter.drawPoint(MM2Px(m_msgWagon.fClusterCenterX[i]), MM2Py(m_msgWagon.fClusterCenterY[i]));
    }

    for (unsigned int i = 0; i < m_msgWagon.fVirtualPointX.size(); i++)
    {
      CWagon_points[i].setX(MM2Px(m_msgWagon.fVirtualPointX[i]));
      CWagon_points[i].setY(MM2Py(m_msgWagon.fVirtualPointY[i]));
    }

    if (m_msgWagon.msgTMSInfo[0].fX != 0 && m_msgWagon.msgTMSInfo[0].fY != 0)
    {
      CPainter.setPen(QPen(Qt::darkRed, 1));
      CPainter.setBrush(Qt::darkRed);
      CPainter.drawPolygon(CWagon_points, m_msgWagon.fVirtualPointX.size());
      // CPainter.drawLine(MM2Px(m_msgWagon.fVirtualPointX[0]), MM2Py(m_msgWagon.fVirtualPointY[0]),
      // MM2Px(m_msgWagon.fVirtualPointX[1]), MM2Py(m_msgWagon.fVirtualPointY[1]));
      // CPainter.drawLine(MM2Px(m_msgWagon.fVirtualPointX[1]), MM2Py(m_msgWagon.fVirtualPointY[1]),
      // MM2Px(m_msgWagon.fVirtualPointX[2]), MM2Py(m_msgWagon.fVirtualPointY[2]));
      // CPainter.drawLine(MM2Px(m_msgWagon.fVirtualPointX[2]), MM2Py(m_msgWagon.fVirtualPointY[2]),
      // MM2Px(m_msgWagon.fVirtualPointX[3]), MM2Py(m_msgWagon.fVirtualPointY[3]));
      // CPainter.drawLine(MM2Px(m_msgWagon.fVirtualPointX[3]), MM2Py(m_msgWagon.fVirtualPointY[3]),
      // MM2Px(m_msgWagon.fVirtualPointX[0]), MM2Py(m_msgWagon.fVirtualPointY[0]));

      // int iRotaionX = (200.0)*cos(fTheta) - (0.0)*sin(fTheta);
      // int iRotaionY = (200.0)*sin(fTheta) + (0.0)*cos(fTheta);

      CPainter.setPen(QPen(Qt::black, 10));
      CPainter.drawPoint(MM2Px(fX), MM2Py(fY));

      if (m_bTagViewWagon == true)
      {
        // CPainter.setPen(QPen(Qt::black, iLineSize*8));
        // CPainter.drawLine(MM2Px(fX), MM2Py(fY), MM2Px(iRotaionX+fX), MM2Py(iRotaionY+fY));

        // CPainter.setPen(QPen(Qt::black, 1));
        // CPainter.drawText(MM2Px(fX), MM2Py(fY+100), "Wagon theta = " + qsTheta.setNum(fTheta * RAD2DEG,'f',0));

        CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
        CPainter.setBrush(Qt::NoBrush);
        QPoint qCenter(MM2Px(fX), MM2Py(fY));
        CPainter.drawEllipse(qCenter, MM2P(WAGON_R), MM2P(WAGON_R));
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawChair(void)
{
  QPainter CPainter(&m_stImage);

  int iLineSize = (int)(m_viewer_scale * 5);

  float fX = 0.0;
  float fY = 0.0;

  QPoint qtpaChairPoints[4] = {QPoint(270, 140), QPoint(-270, 140), QPoint(-270, -140), QPoint(270, -140)};

  fX = m_msgChair.msgTMSInfo[0].fX;
  fY = m_msgChair.msgTMSInfo[0].fY;

  if (m_iViewChair == MODE_CHAIR_ON)
  {
    for (int i = 0; i < m_msgChair.iGroupsCount; i++)
    {
      CPainter.setPen(QPen(Qt::darkYellow, 10));
      CPainter.drawPoint(MM2Px(m_msgChair.fClusterCenterX[i]), MM2Py(m_msgChair.fClusterCenterY[i]));
    }

    for (unsigned int i = 0; i < m_msgChair.fVirtualPointX.size(); i++)
    {
      qtpaChairPoints[i].setX(MM2Px(m_msgChair.fVirtualPointX[i]));
      qtpaChairPoints[i].setY(MM2Py(m_msgChair.fVirtualPointY[i]));
    }

    if (m_msgChair.msgTMSInfo[0].fX != 0 && m_msgChair.msgTMSInfo[0].fY != 0)
    {
      // CPainter.setPen(QPen(Qt::darkGray, 1));
      // CPainter.setBrush(Qt::darkGray);
      // CPainter.drawPolygon(CChair_points,m_msgChair.fVirtualPointX.size());

      CPainter.setPen(QPen(Qt::black, 10));
      CPainter.drawPoint(MM2Px(fX), MM2Py(fY));
    }

    if (m_bTagViewChair == true)
    {
      CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
      CPainter.setBrush(Qt::NoBrush);
      QPoint qCenter(MM2Px(fX), MM2Py(fY));
      CPainter.drawEllipse(qCenter, MM2P(CHAIR_R), MM2P(CHAIR_R));
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawIssWagon(void)
{
  QPainter CPainter(&m_stImage);

  int iLineSize = (int)(m_viewer_scale * 5);

  float fX = 0.0;
  float fY = 0.0;
  float fTheta = 0.0;

  fX = m_msgIssWagon.msgTMSInfo[0].fX;
  fY = m_msgIssWagon.msgTMSInfo[0].fY;
  fTheta = m_msgIssWagon.msgTMSInfo[0].fTheta;

  int iRotationX = (300.0) * cos(fTheta) - (0.0) * sin(fTheta);
  int iRotationY = (300.0) * sin(fTheta) + (0.0) * cos(fTheta);

  if (m_iViewWagonIss == MODE_WAGON_ISS_ON)
  {
    if (fX != 0 && fY != 0)
    {
      CPainter.setPen(QPen(Qt::black, 10));
      CPainter.drawPoint(MM2Px(fX), MM2Py(fY));

      if (m_bTagViewWagonIss == true)
      {
        CPainter.setPen(QPen(Qt::darkBlue, iLineSize * 8));
        CPainter.drawLine(MM2Px(fX), MM2Py(fY), MM2Px(iRotationX + fX), MM2Py(iRotationY + fY));

        CPainter.setPen(QPen(Qt::blue, iLineSize * 2, Qt::DotLine));
        CPainter.setBrush(Qt::NoBrush);
        QPoint qCenter(MM2Px(fX), MM2Py(fY));
        CPainter.drawEllipse(qCenter, MM2P(WAGON_R), MM2P(WAGON_R));
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawIssWheelChair(void)
{
  QPainter CPainter(&m_stImage);

  int iLineSize = (int)(m_viewer_scale * 5);

  float fX = 0.0;
  float fY = 0.0;
  float fTheta = 0.0;

  fX = m_msgIssWheelChair.msgTMSInfo[0].fX;
  fY = m_msgIssWheelChair.msgTMSInfo[0].fY;
  fTheta = m_msgIssWheelChair.msgTMSInfo[0].fTheta;

  int iRotationX = (400.0) * cos(fTheta) - (0.0) * sin(fTheta);
  int iRotationY = (400.0) * sin(fTheta) + (0.0) * cos(fTheta);

  if (m_iViewWheelChair == MODE_WHEELCHAIR_ON)
  {
    if (fX != 0 && fY != 0)
    {
      CPainter.setPen(QPen(Qt::black, 10));
      CPainter.drawPoint(MM2Px(fX), MM2Py(fY));

      if (m_bTagViewWheelChair == true)
      {
        CPainter.setPen(QPen(Qt::darkRed, iLineSize * 8));
        CPainter.drawLine(MM2Px(fX), MM2Py(fY), MM2Px(iRotationX + fX), MM2Py(iRotationY + fY));

        CPainter.setPen(QPen(Qt::red, iLineSize * 2, Qt::DotLine));
        CPainter.setBrush(Qt::NoBrush);
        QPoint qCenter(MM2Px(fX), MM2Py(fY));
        CPainter.drawEllipse(qCenter, MM2P(500), MM2P(500));
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawIcsObject(void)
{
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 70);

  if (m_iViewObject == MODE_OBJECT_ON)
  {
    for (unsigned int i = 0; i < m_msgIcsObjectData.msgTMSInfo.size(); i++)
    {
      if (m_msgIcsObjectData.msgTMSInfo[0].iState != 0)
      {
        // draw intelligent cabinet
        float fX_Offset = 4250;
        float fY_Offset = 850;
        float fX = fX_Offset + m_msgIcsObjectData.msgTMSInfo[i].fY;
        float fY = fY_Offset - m_msgIcsObjectData.msgTMSInfo[i].fX;

        CPainter.setPen(QPen(Qt::black, iPointSize));
        CPainter.drawPoint(MM2Px(fX), MM2Py(fY));

        if (m_bTagViewObject == true)
          CPainter.drawText(MM2Px(4600), MM2Py(fY), m_sObjectName[m_msgIcsObjectData.msgTMSInfo[i].iID].c_str());
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::drawPersonTrajectory(void)
{
  QPainter CPainter(&m_stImage);

  int iPointSize = (int)(m_viewer_scale * 15);
  // CPainter.setPen(QPen(Qt::black, iPointSize));
  QFont font = CPainter.font();
  font.setPointSize(18);
  CPainter.setFont(font);

  if (m_iViewPersonTrajectory == MODE_PERSON_TRAJECTORY_ON)
  {
    int color_id = 6;
    for (unsigned int i = 0; i < m_msgPersonTrajectory.trajectory.size(); i++)
    {
      for (unsigned int j = 0; j < m_msgPersonTrajectory.trajectory[i].fCenterX.size(); j++)
      {
        float this_fCenterX, this_fCenterY;
        float next_fCenterX, next_fCenterY;

        this_fCenterX = m_msgPersonTrajectory.trajectory[i].fCenterX[j];
        this_fCenterY = m_msgPersonTrajectory.trajectory[i].fCenterY[j];

        if (j + 1 < m_msgPersonTrajectory.trajectory[i].fCenterX.size())
        {
          next_fCenterX = m_msgPersonTrajectory.trajectory[i].fCenterX[j + 1];
          next_fCenterY = m_msgPersonTrajectory.trajectory[i].fCenterY[j + 1];

          // line
          CPainter.setPen(QPen(Qt::darkGreen, iPointSize));
          QLineF line(MM2Px(this_fCenterX), MM2Py(this_fCenterY), MM2Px(next_fCenterX), MM2Py(next_fCenterY));
          CPainter.drawLine(line);
        }

        CPainter.setPen(QPen(Qt::darkGreen, iPointSize * 3));
        CPainter.drawPoint(MM2Px(this_fCenterX), MM2Py(this_fCenterY));
      }
      color_id++;
      if (color_id > 14)
        color_id = 6;
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::viewLaser(void)
{
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);

  //--------------------------------------------------------------------------
  if (m_iViewLaser == MODE_LASER_OFF)
  {
  }
  else if (m_iViewLaser == MODE_LASER_POINT)
  {
    if (m_bViewStaticPoint)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_iModeIntensity == MODE_INTENSITY_ON)
        {
          unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                           (m_iMaxIntensityView - m_iMinIntensityView) * 270);

          if (iIntensityHueValue > 270)
            iIntensityHueValue = 270;
          else if (iIntensityHueValue <= 0)
            iIntensityHueValue = 0;

          CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iPointSize * 1.5));
        }
        else
          CPainter.setPen(QPen(Qt::darkGreen, iPointSize));

        int x = MM2Px(m_msgRawdata.fX2[i]);
        int y = MM2Py(m_msgRawdata.fY2[i]);

        CPainter.drawPoint(x, y);
      }
    }

    if (m_bViewDirectPoint)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == false && m_msgRawdata.bIsForwardPoint[i] == true)  // Direct Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iPointSize * 1.5));
          }
          else
            CPainter.setPen(QPen(Qt::red, iPointSize));

          int x = MM2Px(m_msgRawdata.fX2[i]);
          int y = MM2Py(m_msgRawdata.fY2[i]);

          CPainter.drawPoint(x, y);
        }
      }
    }

    if (m_bViewReflectPoint)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == true && m_msgRawdata.bIsForwardPoint[i] == true)  // Reflect Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iPointSize * 1.5));
          }
          else
            CPainter.setPen(QPen(Qt::blue, iPointSize));

          int x = MM2Px(m_msgRawdata.fX2[i]);
          int y = MM2Py(m_msgRawdata.fY2[i]);

          CPainter.drawPoint(x, y);
        }
      }
    }

    if (m_bViewDirectPointAll)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == false)  // Direct Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iPointSize * 1.5));
          }
          else
            CPainter.setPen(QPen(Qt::red, iPointSize));  // size â†’*2 for ICRA, SI, RS

          int x = MM2Px(m_msgRawdata.fX2[i]);
          int y = MM2Py(m_msgRawdata.fY2[i]);

          CPainter.drawPoint(x, y);
        }
      }
    }

    if (m_bViewReflectPointAll)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == true)  // Reflect Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iPointSize * 1.5));
          }
          else
            CPainter.setPen(QPen(Qt::blue, iPointSize));  // blue â†’ red, size â†’*2 for ICRA, SI, RS

          int x = MM2Px(m_msgRawdata.fX2[i]);
          int y = MM2Py(m_msgRawdata.fY2[i]);

          CPainter.drawPoint(x, y);
        }
      }
    }
  }
  else if (m_iViewLaser == MODE_LASER_LINE)  // radioLaserViewLine
  {
    if (m_bViewDirectPoint)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == false && m_msgRawdata.bIsForwardPoint[i] == true)  // Direct Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iLineSize));
          }
          else
            CPainter.setPen(QPen(Qt::red, iLineSize));

          QLineF line(MM2Px(m_dLrf_set_x), MM2Py(m_dLrf_set_y), MM2Px(m_msgRawdata.fX2[i]), MM2Py(m_msgRawdata.fY2[i]));

          CPainter.drawLine(line);
        }
      }
    }

    if (m_bViewReflectPoint)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == true && m_msgRawdata.bIsForwardPoint[i] == true)  // Reflect Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iLineSize));
          }
          else
            CPainter.setPen(QPen(Qt::blue, iLineSize));

          QLineF line1(MM2Px(m_dLrf_set_x), MM2Py(m_dLrf_set_y), MM2Px(m_msgRawdata.fX1[i]),
                       MM2Py(m_msgRawdata.fY1[i]));
          CPainter.drawLine(line1);

          QLineF line2(MM2Px(m_msgRawdata.fX1[i]), MM2Py(m_msgRawdata.fY1[i]), MM2Px(m_msgRawdata.fX2[i]),
                       MM2Py(m_msgRawdata.fY2[i]));
          CPainter.drawLine(line2);
        }
      }
    }

    if (m_bViewDirectPointAll)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == false)  // Direct Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iLineSize));
          }
          else
          {
            if (i == 360)
              CPainter.setPen(QPen(Qt::black, iLineSize));
            else
              CPainter.setPen(QPen(Qt::red, iLineSize));
          }

          QLineF line(MM2Px(m_dLrf_set_x), MM2Py(m_dLrf_set_y), MM2Px(m_msgRawdata.fX2[i]), MM2Py(m_msgRawdata.fY2[i]));
          CPainter.drawLine(line);
        }
      }
    }

    if (m_bViewReflectPointAll)
    {
      for (int i = 0; i < m_dLrf_scan_max_count; i++)
      {
        if (m_msgRawdata.bIsReflect[i] == true)  // Reflect Point
        {
          if (m_iModeIntensity == MODE_INTENSITY_ON)
          {
            unsigned int iIntensityHueValue = (unsigned int)((m_msgRawdata.fIntensity[i] - m_iMinIntensityView) /
                                                             (m_iMaxIntensityView - m_iMinIntensityView) * 270);

            if (iIntensityHueValue > 270)
              iIntensityHueValue = 270;
            else if (iIntensityHueValue <= 0)
              iIntensityHueValue = 0;

            CPainter.setPen(QPen(QColor::fromHsv(iIntensityHueValue, 255, 200), iLineSize));
          }
          else
            CPainter.setPen(QPen(Qt::blue, iLineSize));

          QLineF line1(MM2Px(m_dLrf_set_x), MM2Py(m_dLrf_set_y), MM2Px(m_msgRawdata.fX1[i]),
                       MM2Py(m_msgRawdata.fY1[i]));
          CPainter.drawLine(line1);

          QLineF line2(MM2Px(m_msgRawdata.fX1[i]), MM2Py(m_msgRawdata.fY1[i]), MM2Px(m_msgRawdata.fX2[i]),
                       MM2Py(m_msgRawdata.fY2[i]));
          CPainter.drawLine(line2);
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::viewLaser1(void)
{
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);
  //--------------------------------------------------------------------------
  if (m_iViewLaser == MODE_LASER_OFF)
  {
  }
  else if (m_iViewLaser == MODE_LASER_POINT)
  {
    if (m_bViewStaticPoint)
    {
    }

    if (m_bViewDirectPoint)
    {
    }

    if (m_bViewDirectPointAll)
    {
      CPainter.setPen(QPen(Qt::darkGreen, iPointSize));
      for (int i = 0; i < 512; i++)
      {
        int x = MM2Px(m_msgRawdata1.fDistance[i] * cos(m_dLrf_set_theta1) + m_msgRawdata1.fX2[i]);
        int y = MM2Py(m_msgRawdata1.fDistance[i] * sin(m_dLrf_set_theta1) + m_msgRawdata1.fY2[i]);
        CPainter.drawPoint(x, y);
      }
    }
  }
  else if (m_iViewLaser == MODE_LASER_LINE)  // radioLaserViewLine
  {
    if (m_bViewDirectPoint)
    {
    }

    if (m_bViewDirectPointAll)
    {
      CPainter.setPen(QPen(Qt::darkGreen, iLineSize));
      for (int i = 0; i < 512; i++)
      {
        QLineF line(MM2Px(m_dLrf_set_x1), MM2Py(m_dLrf_set_y1),
                    MM2Px(m_msgRawdata1.fDistance[i] * cos(m_dLrf_set_theta1) + m_msgRawdata1.fX2[i]),
                    MM2Py(m_msgRawdata1.fDistance[i] * sin(m_dLrf_set_theta1) + m_msgRawdata1.fY2[i]));
        CPainter.drawLine(line);
      }
    }
  }
}

//------------------------------------------------------------------------------
void Viewer::viewLaser2(void)
{
  QPainter CPainter(&m_stImage);
  int iPointSize = (int)(m_viewer_scale * 15);
  int iLineSize = (int)(m_viewer_scale * 5);
  //--------------------------------------------------------------------------
  if (m_iViewLaser == MODE_LASER_OFF)
  {
  }
  else if (m_iViewLaser == MODE_LASER_POINT)
  {
    if (m_bViewStaticPoint)
    {
    }

    if (m_bViewDirectPoint)
    {
    }

    if (m_bViewDirectPointAll)
    {
      CPainter.setPen(QPen(Qt::blue, iPointSize));
      for (int i = 0; i < 512; i++)
      {
        int x = MM2Px(m_msgRawdata2.fDistance[i] * cos(m_dLrf_set_theta2) + m_msgRawdata2.fX2[i]);
        int y = MM2Py(m_msgRawdata2.fDistance[i] * sin(m_dLrf_set_theta2) + m_msgRawdata2.fY2[i]);
        CPainter.drawPoint(x, y);
      }
    }
  }
  else if (m_iViewLaser == MODE_LASER_LINE)  // radioLaserViewLine
  {
    if (m_bViewDirectPoint)
    {
    }

    if (m_bViewDirectPointAll)
    {
      CPainter.setPen(QPen(Qt::blue, iLineSize));
      for (int i = 0; i < 512; i++)
      {
        QLineF line(MM2Px(m_dLrf_set_x2), MM2Py(m_dLrf_set_y2),
                    MM2Px(m_msgRawdata2.fDistance[i] * cos(m_dLrf_set_theta2) + m_msgRawdata2.fX2[i]),
                    MM2Py(m_msgRawdata2.fDistance[i] * sin(m_dLrf_set_theta2) + m_msgRawdata2.fY2[i]));
        CPainter.drawLine(line);
      }
    }
  }
}
