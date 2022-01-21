//------------------------------------------------------------------------------
// @file   : viewer.h
// @brief  : viewer_setting function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------

#ifndef VIEWER_H
#define VIEWER_H

#ifndef Q_MOC_RUN

//------------------------------------------------------------------------------
#include <QWidget>
#include <QtGui>

#include <ros/ros.h>

#include "mainwindow.h"
#include "qnode.hpp"
#include <tms_msg_ss/tracking_points.h>
#include <tms_msg_ss/tracking_grid.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>

#define ORIGIN_X 77   // 32.0
#define ORIGIN_Y 841  // 724.0
#define GRID_X_SIZE 15
#define GRID_Y_SIZE 7
#define REDUCED_SCALE 113.103448276  // 97.2413793103

class MainWindow;

//------------------------------------------------------------------------------
class Viewer : public QWidget
{
  Q_OBJECT

public:
  double test_count;

  bool clearImage_Button;
  bool reloadImage_Button;
  bool track_Nothing_button;
  bool track_Gradual_disappear;
  bool track_All_remind_button;
  bool grid_on;
  bool grid_off;
  bool laser_on;
  bool laser_off;
  bool laser_key;

  struct tracker_param
  {
    int id;
    int flag;
    int count;
    double before_x;
    double before_y;
    double x;
    double y;
    double after_x;
    double after_y;
  };

  struct tracker_point
  {
    double x;
    double y;
  };

  struct tracker_param2
  {
    int id;
    int flag;
    int count;
    std::vector< tracker_point > xy;
    double start_time;
    std::vector< double > xy_time;
  };

  tracker_param tmp_tracker_info;
  std::vector< tracker_param > tracker_info_array;
  std::vector< tracker_param >::iterator p;

  tracker_param2 tmp_tracker_info2;
  std::vector< tracker_param2 > tracker_info_array2;
  std::vector< tracker_param2 >::iterator p2;

  Viewer(QNode *node, QWidget *parent = 0);
  virtual ~Viewer();

public Q_SLOTS:
  void view();
  void drawLine(double s_x, double s_y, double e_x, double e_y, QPen pen);
  void drawPoint(double x, double y, QPen pen);
  void drawGridSquare();
  void drawHumanPoint();
  void drawHumanLine();
  void drawGradualHumanLine();
  void Bezier_Curve(tracker_point before, tracker_point now, tracker_point now2, tracker_point after);

  void laserDraw1();
  void laserDraw2();
  void laserDraw3();

  void setPortable(int x, int y);

protected:
  void clearImage();
  void reloadImage();
  void plotWriteFIle();
  void paintEvent(QPaintEvent *event);
  void resizeEvent(QResizeEvent *event);

private:
  void Callback(const tms_msg_ss::tracking_points::ConstPtr &msg);
  void laser_Callback1(const sensor_msgs::LaserScan::ConstPtr &scan1);
  void laser_Callback2(const sensor_msgs::LaserScan::ConstPtr &scan2);
  void laser_Callback3(const sensor_msgs::LaserScan::ConstPtr &scan3);

  ros::Subscriber Sub;
  ros::Subscriber laser_sub1;
  ros::Subscriber laser_sub2;
  ros::Subscriber laser_sub3;
  tms_msg_ss::tracking_points msgRawdata;
  sensor_msgs::LaserScan laserRawdata1;
  sensor_msgs::LaserScan laserRawdata2;
  sensor_msgs::LaserScan laserRawdata3;

  QLabel label;
  QImage m_stImage;
  QTimer *m_timer;
  MainWindow *main;
  QNode *qnode;
};

#endif
//------------------------------------------------------------------------------
#endif  // VIEWER_H
//------------------------------------------------------------------------------
