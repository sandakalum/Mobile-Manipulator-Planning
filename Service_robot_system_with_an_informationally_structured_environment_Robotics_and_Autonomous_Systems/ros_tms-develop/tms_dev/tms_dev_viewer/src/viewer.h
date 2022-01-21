#ifndef VIEWER_H
#define VIEWER_H

//------------------------------------------------------------------------------
#include <QWidget>
#include <QtGui>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>

#include "common.h"
#include "mainwindow.h"
#include "qnode.hpp"

#include <tms_msg_ss/fss_pre_data.h>
#include <tms_msg_ss/fss_tf_data.h>
#include <tms_msg_ss/fss_cluster_data.h>
#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_ss/fss_object_data.h>
#include <tms_msg_ss/fss_person_trajectory_data.h>
#include <tms_msg_ss/fss_observed_datas.h>
#include <tms_msg_ss/ics_object_data.h>
#include <tms_msg_ss/iss_object_data.h>
#include <tms_msg_db/tmsdb_data.h>
#include <tms_msg_db/tmsdb_robot_trajectory_data.h>
#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_map_data.h>
#include <tms_msg_rp/rps_map_y.h>
#include <tms_msg_rp/rps_map_full.h>

//------------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846  // pi
#endif

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)

#define ANGULAR_RESOLUTION 721  // 721 * 0.25 = 180.25 , 181

//------------------------------------------------------------------------------
class MainWindow;

//------------------------------------------------------------------------------
class Viewer : public QWidget
{
  Q_OBJECT

public:
  Viewer(QNode* node, QWidget* parent = 0);

  double m_viewer_scale;

  int m_iModeGrid;

  bool m_bViewStaticPoint;
  bool m_bViewDirectPoint;
  bool m_bViewReflectPoint;
  bool m_bViewDirectPointAll;
  bool m_bViewReflectPointAll;

  int m_iViewLaser;
  int m_iViewOcclusion;

  int m_iViewCluster;
  bool m_bTagViewCluster;

  int m_iViewClass;
  bool m_bTagViewClass;
  bool m_bViewTwoColors;
  int m_iIntensityThreshold;

  int m_iViewUnknownClass;
  bool m_bTagViewUnknownClass;
  bool m_bViewUnknownTwoColors;
  int m_iUnknownIntensityThreshold;

  int m_iViewFurniture;
  bool m_bTagViewFurniture;

  int m_iViewPath;
  bool m_bPlanPathView;
  bool m_bRobotPathView;
  bool m_bWagonPathView;

  int m_iViewRPS_MAP;
  bool m_bCollisionAreaViewRPS_MAP;
  bool m_bvoronoiLineViewRPS_MAP;

  int m_iViewSmartpal;
  bool m_bSmartpalTrajectory;
  bool m_bTagViewSmartpal;
  int m_iViewRoomba;
  bool m_bRoombaTrajectory;
  bool m_bTagViewRoomba;

  ros::Time m_tSmartpalTrajectoryStartTime;
  ros::Time m_tRoombaTrajectoryStartTime;

  int m_iViewWagon;
  bool m_bTagViewWagon;
  int m_iViewChair;
  bool m_bTagViewChair;

  int m_iViewPersonTrajectory;
  bool m_bTagViewPersonTrajectory;

  int m_iViewUnknownObject;
  bool m_bTagViewUnknownObject;

  int m_iViewWagonIss;
  bool m_bTagViewWagonIss;
  int m_iViewWheelChair;
  bool m_bTagViewWheelChair;

  int m_iViewObject;
  bool m_bTagViewObject;

  int m_iModeIntensity;
  bool m_bTagViewIntensity;

  int m_iDataViewRowIndex;

  int m_dLrf_scan_max_count;
  LRF m_dUtm30LX_raw_data[ANGULAR_RESOLUTION];

  int m_iMinIntensityView;
  int m_iMaxIntensityView;

public Q_SLOTS:
  void clearImage();
  void view();

protected:
  void keyPressEvent(QKeyEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void wheelEvent(QWheelEvent* event);
  void paintEvent(QPaintEvent* event);
  void resizeEvent(QResizeEvent* event);

private:
  int MM2P(double f_dP);
  int MM2Px(double f_dX);
  int MM2Py(double f_dY);
  int MM2Pl(double f_dD);
  double P2MMx(int f_iX);
  double P2MMy(int f_iY);
  double P2MMl(int f_iD);

  void drawMap();
  void drawGridSquare();
  void drawGridCircle();

  void drawPlanPath();
  void drawRobotPath();
  void drawWagonPath();
  void drawRPS_MAP();

  void drawFurniture();
  void drawSmartpal(float f_dX, float f_dY, float f_dTheta);
  bool drawSmartpalTrajectory(ros::Time tStart, ros::Time tEnd);
  void drawRoomba(float f_dX, float f_dY, float f_dTheta);
  bool drawRoombaTrajectory(ros::Time tStart, ros::Time tEnd);
  void drawWagon();
  void drawChair();
  void drawPersonTrajectory();
  void drawIssWagon();
  void drawIssWheelChair();
  void drawIcsObject();

  void init_lrf();

  void load_map();
  void load_grid();

  void fssPreDataCallback(const tms_msg_ss::fss_pre_data::ConstPtr& msg);
  void fssTfDataCallback(const tms_msg_ss::fss_tf_data::ConstPtr& msg);
  void fssTfDataCallback1(const tms_msg_ss::fss_tf_data::ConstPtr& msg);
  void fssTfDataCallback2(const tms_msg_ss::fss_tf_data::ConstPtr& msg);
  void fssClusterDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg);
  void fssClassDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg);
  void fssUnknownClassDataCallback(const tms_msg_ss::fss_cluster_data::ConstPtr& msg);
  void rpsPlanPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg);
  void rpsRobotPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg);
  void rpsWagonPathCallback(const tms_msg_rp::rps_route::ConstPtr& msg);
  void RPS_MAPCallback(const tms_msg_rp::rps_map_full::ConstPtr& msg);
  void fssSmartpalDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg);
  void fssRoombaDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg);
  void fssWagonDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg);
  void fssChairDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg);
  void fssPersonTrajectoryDataCallback(const tms_msg_ss::fss_person_trajectory_data::ConstPtr& msg);
  void fssUnknownObjectDataCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg);
  void issWagonDataCallback(const tms_msg_ss::iss_object_data::ConstPtr& msg);
  void issWheelChairDataCallback(const tms_msg_ss::iss_object_data::ConstPtr& msg);
  void icsObjectDataCallback(const tms_msg_ss::ics_object_data::ConstPtr& msg);

  void display();
  void viewLaser();
  void viewLaser1();
  void viewLaser2();

  bool m_bOnce;

  bool m_bScribbling;
  bool m_bMoving;
  int m_iSelect_object;

  double m_dMouse_point_x;
  double m_dMouse_point_y;

  double m_dOffset_x;
  double m_dOffset_y;

  bool m_bIs_making_background;
  bool m_bIs_background;
  double m_dWindow_size_x;
  double m_dWindow_size_y;

  double m_dLrf_set_x;
  double m_dLrf_set_y;
  double m_dLrf_set_theta;

  double m_dLrf_scan_max_range;
  double m_dLrf_scan_max_distance;

  double m_dLrf_set_x1;
  double m_dLrf_set_y1;
  double m_dLrf_set_theta1;
  double m_dLrf_set_x2;
  double m_dLrf_set_y2;
  double m_dLrf_set_theta2;

  QImage m_stImage;

  QPoint m_stLast_point;

  vector< LINE > m_vstMap_2d;
  vector< LINE > m_vstGrid_2d;
  vector< LINE > m_vstRobot_smartpal_2d;
  vector< LINE > m_vstRobot_roomba_2d;
  vector< LINE > m_vstRobot_pioneer_2d;

  int m_iCallback;

  ros::Subscriber fss_pre_data_subscriber;
  ros::Subscriber fss_tf_data_subscriber;
  ros::Subscriber fss_tf_data_subscriber1;
  ros::Subscriber fss_tf_data_subscriber2;
  ros::Subscriber fss_cluster_data_subscriber;
  ros::Subscriber fss_class_data_subscriber;
  ros::Subscriber fss_Unknown_class_data_subscriber;
  ros::Subscriber fss_smartpal_data_subscriber;
  ros::Subscriber fss_roomba_data_subscriber;
  ros::Subscriber fss_wagon_data_subscriber;
  ros::Subscriber fss_chair_data_subscriber;
  ros::Subscriber fss_person_trajectory_data_subscriber;
  ros::Subscriber fss_unknown_object_data_subscriber;
  ros::Subscriber iss_wagon_data_subscriber;
  ros::Subscriber iss_wheelchair_data_subscriber;
  ros::Subscriber ics_object_data_subscriber;

  ros::Subscriber rps_plan_path_subscriber;
  ros::Subscriber rps_robot_path_subscriber;
  ros::Subscriber rps_wagon_path_subscriber;
  ros::Subscriber rps_MAP_subscriber;

  ros::ServiceClient rosclientTmsdbRobotTrajectoryData;

  QTimer* m_timer;
  MainWindow* main;
  QNode* qnode;

  QRgb qColorID[21];
  string m_sObjectName[61];

  tms_msg_ss::fss_tf_data m_msgRawdata;
  tms_msg_ss::fss_tf_data m_msgRawdata1;
  tms_msg_ss::fss_tf_data m_msgRawdata2;
  tms_msg_ss::fss_cluster_data m_msgCluster;
  tms_msg_ss::fss_cluster_data m_msgClass;
  tms_msg_ss::fss_cluster_data m_msgUnknownClass;
  tms_msg_ss::fss_object_data m_msgSmartpal;
  tms_msg_ss::fss_object_data m_msgRoomba;
  tms_msg_ss::fss_object_data m_msgWagon;
  tms_msg_ss::fss_object_data m_msgChair;
  tms_msg_ss::fss_object_data m_msgUnknownObject;
  tms_msg_ss::iss_object_data m_msgIssWagon;
  tms_msg_ss::iss_object_data m_msgIssWheelChair;
  tms_msg_ss::ics_object_data m_msgIcsObjectData;
  tms_msg_ss::fss_person_trajectory_data m_msgPersonTrajectory;

  tms_msg_db::tmsdb_robot_trajectory_data m_srvSmartpalTrajectory;
  tms_msg_db::tmsdb_robot_trajectory_data m_srvRoombaTrajectory;

  tms_msg_rp::rps_route m_msgPlanPath;
  tms_msg_rp::rps_route m_msgRobotPath;
  tms_msg_rp::rps_route m_msgWagonPath;
  tms_msg_rp::rps_map_full m_msgRPS_MAP;

  bool isPersonDataAllRecevied;
  int personVectorSwitcher;
};

//------------------------------------------------------------------------------
#endif  // VIEWER_H

//------------------------------------------------------------------------------
