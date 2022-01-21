//------------------------------------------------------------------------------
// @file   : viewer.cpp
// @brief  : viewer_setting function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------

#include <QtGui>
#include <QPainter>

#include "viewer.h"
#include "qnode.hpp"
#include "iostream"
#include "std_msgs/String.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "opencv/cv.h"

#define PI 3.1415926535897932384626433832795
#define pv 0.6666666666666666
#define vp 0.3333333333333333
#define GRAB_POINT 40
#define WINDOW_TIME 10.0  // 3.0//4.0//3.0

namespace fs = boost::filesystem;

void Viewer::laser_Callback1(const sensor_msgs::LaserScan::ConstPtr &scan1)
{
  laserRawdata1 = *scan1;
}

void Viewer::laser_Callback2(const sensor_msgs::LaserScan::ConstPtr &scan2)
{
  laserRawdata2 = *scan2;
}

void Viewer::laser_Callback3(const sensor_msgs::LaserScan::ConstPtr &scan3)
{
  laserRawdata3 = *scan3;
}

void Viewer::Callback(const tms_msg_ss::tracking_points::ConstPtr &msg)
{
  msgRawdata = *msg;
}

Viewer::Viewer(QNode *node, QWidget *parent) : QWidget(parent), test_count(0.0), qnode(node)
{
  //-----------------------------------------------------------------------
  // ros_setting
  //-----------------------------------------------------------------------
  ros::init(qnode->init_argc, qnode->init_argv, qnode->node_name);
  ros::NodeHandle nh;
  Sub = nh.subscribe("tracking_points", 10, &Viewer::Callback, this);
  laser_sub1 = nh.subscribe("LaserTracker1", 10, &Viewer::laser_Callback1, this);
  laser_sub2 = nh.subscribe("LaserTracker2", 10, &Viewer::laser_Callback2, this);
  laser_sub3 = nh.subscribe("LaserTracker3", 10, &Viewer::laser_Callback3, this);

  //-----------------------------------------------------------------------
  // draw_setting
  //-----------------------------------------------------------------------
  m_stImage.load("./src/ros_tms/tms_dev/portable_viewer/image/COI3.jpg");
  drawGridSquare();
  clearImage_Button = false;
  reloadImage_Button = false;
  track_Nothing_button = false;
  track_Gradual_disappear = false;
  track_All_remind_button = false;
  grid_on = false;
  grid_off = false;
  laser_on = false;
  laser_off = false;
  laser_key = false;

  //-----------------------------------------------------------------------
  // Portable place setting
  //-----------------------------------------------------------------------
  double laser_x = 7.75;
  double laser_y = 5.55;
  double s_X = ORIGIN_X + laser_x * REDUCED_SCALE;
  double s_Y = ORIGIN_Y - laser_y * REDUCED_SCALE;
  setPortable((int)s_X, (int)s_Y);

  laser_x = 11.50;
  laser_y = 4.0;
  s_X = ORIGIN_X + laser_x * REDUCED_SCALE;
  s_Y = ORIGIN_Y - laser_y * REDUCED_SCALE;
  setPortable((int)s_X, (int)s_Y);

  laser_x = 8.55;
  laser_y = 0.15;
  s_X = ORIGIN_X + laser_x * REDUCED_SCALE;
  s_Y = ORIGIN_Y - laser_y * REDUCED_SCALE;
  setPortable((int)s_X, (int)s_Y);

  //-----------------------------------------------------------------------
  // QTimer_setting
  //-----------------------------------------------------------------------
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(view()));
  timer->start(1);
}

Viewer::~Viewer()
{
}

void Viewer::view()
{
  //-----------------------------------------------------------------------
  // write plot to txt.data
  //-----------------------------------------------------------------------
  // plotWriteFIle();

  //-----------------------------------------------------------------------
  // Draw_track_image
  //-----------------------------------------------------------------------
  // if(track_Nothing_button)
  // {
  //   drawHumanPoint();
  // }
  // else if(track_All_remind_button)
  // {
  //   drawHumanLine();
  // }
  // if(this->track_Gradual_disappear)
  // {
  drawGradualHumanLine();
  //}

  //-----------------------------------------------------------------------
  // Image_viewer_button
  //-----------------------------------------------------------------------
  // if(clearImage_Button)
  // {
  //   clearImage();
  // }
  // else if(reloadImage_Button)
  // {
  //   reloadImage();
  // }

  //-----------------------------------------------------------------------
  // grid_viewer_button
  //-----------------------------------------------------------------------
  // if(grid_on)
  // {
  //    drawGridSquare();
  // }

  if (this->laser_on)
  {
    clearImage();
    laserDraw1();
    laserDraw2();
    laserDraw3();
    update();
    this->track_Nothing_button = false;
    this->track_Gradual_disappear = false;
    this->laser_key = true;
  }
  else if (this->laser_off && this->laser_key)
  {
    clearImage();
    this->laser_key = false;
  }
  ros::spinOnce();
}

void Viewer::laserDraw1()
{
  double laser_x = 7.75;
  double laser_y = 5.55;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0, cos(-PI / 2.0));
  cvmSet(m_Rotate, 0, 1, -sin(-PI / 2.0));
  cvmSet(m_Rotate, 1, 0, sin(-PI / 2.0));
  cvmSet(m_Rotate, 1, 1, cos(-PI / 2.0));

  for (unsigned int i = 0; i < laserRawdata1.ranges.size(); i++)
  {
    theta = laserRawdata1.angle_increment * i + laserRawdata1.angle_min;
    cvmSet(Temp, 0, 0, laserRawdata1.ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, laserRawdata1.ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;
    drawLine(laser_x, laser_y, laser_point_x, laser_point_y, QPen(Qt::red, 1));
  }
}

void Viewer::laserDraw2()
{
  double laser_x = 11.50;
  double laser_y = 4.0;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0, cos(0.0));
  cvmSet(m_Rotate, 0, 1, -sin(0.0));
  cvmSet(m_Rotate, 1, 0, sin(0.0));
  cvmSet(m_Rotate, 1, 1, cos(0.0));

  for (unsigned int i = 0; i < laserRawdata2.ranges.size(); i++)
  {
    theta = laserRawdata2.angle_increment * i + laserRawdata2.angle_min;
    cvmSet(Temp, 0, 0, laserRawdata2.ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, laserRawdata2.ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;
    drawLine(laser_x, laser_y, laser_point_x, laser_point_y, QPen(Qt::blue, 1));
  }
}

void Viewer::laserDraw3()
{
  double laser_x = 8.55;
  double laser_y = 0.15;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0, cos(PI / 2));
  cvmSet(m_Rotate, 0, 1, -sin(PI / 2));
  cvmSet(m_Rotate, 1, 0, sin(PI / 2));
  cvmSet(m_Rotate, 1, 1, cos(PI / 2));

  for (unsigned int i = 0; i < laserRawdata3.ranges.size(); i++)
  {
    theta = laserRawdata3.angle_increment * i + laserRawdata3.angle_min;
    cvmSet(Temp, 0, 0, laserRawdata3.ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, laserRawdata3.ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;
    drawLine(laser_x, laser_y, laser_point_x, laser_point_y, QPen(Qt::green, 1));
  }
}

void Viewer::clearImage()
{
  // m_stImage.load("src/ros_tms/tms_dev/test_viewer/image/COI.jpg");
  m_stImage.load("./src/ros_tms/tms_dev/portable_viewer/image/COI3.jpg");
  this->clearImage_Button = false;
  update();
}

void Viewer::reloadImage()
{
  const fs::path path("/home/tb/catkin_ws/psen_demo");
  BOOST_FOREACH (const fs::path &p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator()))
  {
    if (!fs::is_directory(p) && boost::algorithm::iends_with(p.string(), ".txt"))
    {
      std::ifstream ifs(p.c_str());
      std::string line;
      double s_x, s_y, e_x, e_y;

      int fp = 0;
      int fn = 0;
      int line_count = 0;

      while (getline(ifs, line))
      {
        fn = line.find(",", fp);
        fn = fn - fp;
        if (line_count == 0)
        {
          s_x = atof(line.substr(fp, fn).c_str());
        }
        if (line_count == 1)
        {
          e_x = atof(line.substr(fp, fn).c_str());
        }
        if (line_count % GRAB_POINT == 0 && line_count != 0)
        {
          s_x = e_x;
          e_x = atof(line.substr(fp, fn).c_str());
        }
        fp = fp + fn + 1;
        fn = line.find(",", fp);
        fn = fn - fp;
        if (line_count == 0)
        {
          s_y = atof(line.substr(fp, fn).c_str());
        }
        if (line_count == 1)
        {
          e_y = atof(line.substr(fp, fn).c_str());
          // drawLine(s_x, s_y ,e_x ,e_y);
        }
        if (line_count % GRAB_POINT == 0 && line_count != 0)
        {
          s_y = e_y;
          e_y = atof(line.substr(fp, fn).c_str());
          // drawLine(s_x, s_y ,e_x ,e_y);
        }
        fp = 0;
        fn = 0;
        line_count++;
      }
    }
  }
  this->reloadImage_Button = false;
}

void Viewer::plotWriteFIle()
{
  for (int i = 0; i < msgRawdata.tracking_grid.size(); i++)
  {
    int id = msgRawdata.tracking_grid[i].id;
    double x = msgRawdata.tracking_grid[i].x;
    double y = msgRawdata.tracking_grid[i].y;

    char str[20];
    sprintf(str, "psen_demo/ID%d.txt", id);
    std::ofstream strw;
    strw.open(str, std::ofstream::out | std::ofstream::app);
    strw << x << "," << y << "," << std::endl;
  }
}

void Viewer::paintEvent(QPaintEvent *event)
{
  QPainter CPainter(this);
  QRect dirtyRect = event->rect();
  CPainter.drawImage(dirtyRect, m_stImage, dirtyRect);
}

void Viewer::resizeEvent(QResizeEvent *event)
{
  QImage newImage(event->size(), QImage::Format_RGB32);
  // newImage.load("src/ros_tms/tms_dev/test_viewer/image/COI.jpg");
  newImage.load("./src/ros_tms/tms_dev/portable_viewer/image/COI3.jpg");
  m_stImage = newImage;
}

void Viewer::drawLine(double s_x, double s_y, double e_x, double e_y, QPen pen)
{
  double s_X, s_Y, e_X, e_Y;
  QPainter CPainter(&m_stImage);
  CPainter.setPen(pen);
  s_X = ORIGIN_X + s_x * REDUCED_SCALE;
  s_Y = ORIGIN_Y - s_y * REDUCED_SCALE;
  e_X = ORIGIN_X + e_x * REDUCED_SCALE;
  e_Y = ORIGIN_Y - e_y * REDUCED_SCALE;
  QLineF line(s_X, s_Y, e_X, e_Y);
  CPainter.drawLine(line);
}

void Viewer::drawPoint(double x, double y, QPen pen)
{
  double X, Y;
  QPainter CPainter(&m_stImage);
  CPainter.setPen(pen);
  X = ORIGIN_X + x * REDUCED_SCALE;
  Y = ORIGIN_Y - y * REDUCED_SCALE;
  QPoint point(X, Y);
  CPainter.drawPoint(point);
}

void Viewer::drawGridSquare()
{
  double s_X, s_Y, e_X, e_Y;
  QPainter CPainter(&m_stImage);
  CPainter.setPen(QPen(Qt::lightGray, 1));
  // DRAW X_GRID
  for (unsigned int i = 0; i < GRID_X_SIZE + 1; i++)
  {
    s_X = ORIGIN_X + i * REDUCED_SCALE;
    s_Y = ORIGIN_Y;
    e_X = ORIGIN_X + i * REDUCED_SCALE;
    e_Y = ORIGIN_Y - (GRID_Y_SIZE * REDUCED_SCALE);
    QLineF line(s_X, s_Y, e_X, e_Y);
    CPainter.drawLine(line);
  }
  // DRAW Y_GRID
  for (unsigned int i = 0; i < GRID_Y_SIZE + 1; i++)
  {
    s_X = ORIGIN_X;
    s_Y = ORIGIN_Y - i * REDUCED_SCALE;
    e_X = ORIGIN_X + (GRID_X_SIZE * REDUCED_SCALE);
    e_Y = ORIGIN_Y - i * REDUCED_SCALE;
    QLineF line(s_X, s_Y, e_X, e_Y);
    CPainter.drawLine(line);
  }
  update();
}

void Viewer::drawHumanPoint()
{
  clearImage();
  if (msgRawdata.tracking_grid.size() > 0)
  {
    for (int i = 0; i < msgRawdata.tracking_grid.size(); i++)
    {
      double x = msgRawdata.tracking_grid[i].x;
      double y = msgRawdata.tracking_grid[i].y;

      if ((msgRawdata.tracking_grid[i].id) % 7 == 0)
      {
        drawPoint(x, y, QPen(Qt::red, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 1)
      {
        drawPoint(x, y, QPen(Qt::blue, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 2)
      {
        drawPoint(x, y, QPen(Qt::green, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 3)
      {
        drawPoint(x, y, QPen(Qt::cyan, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 4)
      {
        drawPoint(x, y, QPen(Qt::gray, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 5)
      {
        drawPoint(x, y, QPen(Qt::magenta, 15));
      }
      else if ((msgRawdata.tracking_grid[i].id) % 7 == 6)
      {
        drawPoint(x, y, QPen(Qt::yellow, 15));
      }
    }
  }
  update();
}

void Viewer::drawHumanLine()
{
  for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
  {
    // all flag -> 0
    p->flag = 0;
  }
  if (msgRawdata.tracking_grid.size() > 0)
  {
    for (int i = 0; i < msgRawdata.tracking_grid.size(); i++)
    {
      if (tracker_info_array.size() > 0)
      {
        for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
        {
          if (p->id == msgRawdata.tracking_grid[i].id)
          {
            p->count++;
            // candidate on the array

            if (p->count >= 4 && p->count % GRAB_POINT == 0)
            {
              tracker_point before_point;
              tracker_point now_point;
              tracker_point now2_point;
              tracker_point after_point;
              before_point.x = p->before_x;
              before_point.y = p->before_y;
              now_point.x = p->x;
              now_point.y = p->y;
              now2_point.x = p->after_x;
              now2_point.y = p->after_y;
              after_point.x = msgRawdata.tracking_grid[i].x;
              after_point.y = msgRawdata.tracking_grid[i].y;

              // Bezier_Curve(before_point, now_point, now2_point, after_point);
              // drawLine(p -> x, p -> y ,p -> after_x ,p -> after_y);

              p->before_x = p->x;
              p->before_y = p->y;
              p->x = p->after_x;
              p->y = p->after_y;
              p->after_x = msgRawdata.tracking_grid[i].x;
              p->after_y = msgRawdata.tracking_grid[i].y;
            }
            else if (p->count <= 4)
            {
              p->flag = 1;
              p->before_x = p->x;
              p->before_y = p->y;
              p->x = p->after_x;
              p->y = p->after_y;
              p->after_x = msgRawdata.tracking_grid[i].x;
              p->after_y = msgRawdata.tracking_grid[i].y;
            }

            p->flag = 1;
            p->after_x = msgRawdata.tracking_grid[i].x;
            p->after_y = msgRawdata.tracking_grid[i].y;
            break;
          }
          else if (p == tracker_info_array.end() - 1)
          {
            // candidate not on the array
            tmp_tracker_info.id = msgRawdata.tracking_grid[i].id;
            tmp_tracker_info.flag = 1;
            tmp_tracker_info.count = 0;
            tmp_tracker_info.before_x = msgRawdata.tracking_grid[i].x;
            tmp_tracker_info.before_y = msgRawdata.tracking_grid[i].y;
            tmp_tracker_info.x = msgRawdata.tracking_grid[i].x;
            tmp_tracker_info.y = msgRawdata.tracking_grid[i].y;
            tracker_info_array.insert(tracker_info_array.begin(), tmp_tracker_info);
            break;
          }
        }
      }
      else
      {
        // no array
        tmp_tracker_info.id = msgRawdata.tracking_grid[i].id;
        tmp_tracker_info.flag = 1;
        tmp_tracker_info.count = 0;
        tmp_tracker_info.before_x = msgRawdata.tracking_grid[i].x;
        tmp_tracker_info.before_y = msgRawdata.tracking_grid[i].y;
        tmp_tracker_info.x = msgRawdata.tracking_grid[i].x;
        tmp_tracker_info.y = msgRawdata.tracking_grid[i].y;
        tracker_info_array.push_back(tmp_tracker_info);
      }
    }
  }
  if (tracker_info_array.size() > 0)
  {
    tracker_info_array.resize(tracker_info_array.size());
  L:
    int n_flag = 0;
    for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
    {
      if (p->flag == 0)
      {
        tracker_info_array.erase(p);
        n_flag = 1;
        break;
      }
    }
    if (n_flag == 1)
    {
      goto L;
    }
  }
  update();
}

void Viewer::Bezier_Curve(tracker_point before, tracker_point now, tracker_point now2, tracker_point after)
{
  double p0_x, p1_x, p2_x, p3_x;
  double p0_y, p1_y, p2_y, p3_y;
  double t;
  double p_x, p_y;
  double p_before_x, p_before_y;
  // drawLine
  QPainter CPainter(&m_stImage);
  CPainter.setPen(QPen(Qt::black, 1));

  p0_x = before.x;
  p0_y = before.y;
  p3_x = after.x;
  p3_y = after.y;

  p1_x = (now.x - ((pv) * (pv) * (pv)) * p0_x - (3 * (pv) * (vp) * (vp)) * now2.x - ((vp) * (vp) * (vp)*p3_x)) /
         (3 * (pv) * (pv) * (vp));
  p1_y = (now.y - ((pv) * (pv) * (pv)) * p0_y - (3 * (pv) * (vp) * (vp)) * now2.y - ((vp) * (vp) * (vp)*p3_y)) /
         (3 * (pv) * (pv) * (vp));

  p2_x = (now2.x - ((vp) * (vp) * (vp)) * p0_x - (3 * (vp) * (vp) * (pv)) * now.x - ((pv) * (pv) * (pv)*p3_x)) /
         (3 * (pv) * (pv) * (vp));
  p2_y = (now2.y - ((vp) * (vp) * (vp)) * p0_y - (3 * (vp) * (vp) * (pv)) * now.y - ((pv) * (pv) * (pv)*p3_y)) /
         (3 * (pv) * (pv) * (vp));

  p1_x = (now.x - ((pv) * (pv) * (pv)) * p0_x - (3 * (pv) * (vp) * (vp)) * p2_x - ((vp) * (vp) * (vp)*p3_x)) /
         (3 * (pv) * (pv) * (vp));
  p1_y = (now.y - ((pv) * (pv) * (pv)) * p0_y - (3 * (pv) * (vp) * (vp)) * p2_y - ((vp) * (vp) * (vp)*p3_y)) /
         (3 * (pv) * (pv) * (vp));

  // initialize
  t = 0.0;
  p_before_x = ORIGIN_X + p0_x * REDUCED_SCALE;
  p_before_y = ORIGIN_Y - p0_y * REDUCED_SCALE;
  while (1)
  {
    p_x = (1.0 - t) * (1.0 - t) * (1.0 - t) * p0_x + 3 * (1.0 - t) * (1.0 - t) * t * p1_x +
          3 * (1.0 - t) * t * t * p2_x + t * t * t * p3_x;
    p_y = (1.0 - t) * (1.0 - t) * (1.0 - t) * p0_y + 3 * (1.0 - t) * (1.0 - t) * t * p1_y +
          3 * (1.0 - t) * t * t * p2_y + t * t * t * p3_y;
    p_x = ORIGIN_X + p_x * REDUCED_SCALE;
    p_y = ORIGIN_Y - p_y * REDUCED_SCALE;
    QLineF line(p_before_x, p_before_y, p_x, p_y);
    CPainter.drawLine(line);
    p_before_x = p_x;
    p_before_y = p_y;
    t = t + 0.01;

    if (t >= 0.333)
      break;
  }
}

void Viewer::drawGradualHumanLine()
{
  clearImage();
  for (p2 = tracker_info_array2.begin(); p2 != tracker_info_array2.end(); ++p2)
  {
    // all flag -> 0
    p2->flag = 0;
  }
  if (msgRawdata.tracking_grid.size() > 0)
  {
    for (int i = 0; i < msgRawdata.tracking_grid.size(); i++)
    {
      if (tracker_info_array2.size() > 0)
      {
        for (p2 = tracker_info_array2.begin(); p2 != tracker_info_array2.end(); ++p2)
        {
          if (p2->id == msgRawdata.tracking_grid[i].id)
          {
            p2->count++;
            // candidate on the array
            if (p2->count % GRAB_POINT == 0)
            {
              tracker_point tmp_xy;
              tmp_xy.x = msgRawdata.tracking_grid[i].x;
              tmp_xy.y = msgRawdata.tracking_grid[i].y;
              p2->xy.push_back(tmp_xy);
              p2->xy_time.push_back(ros::Time::now().toSec());
            }
            p2->flag = 1;
            std::vector< tracker_point >::iterator vv;
            vv = (p2->xy).begin();

            std::vector< double >::iterator vvv;
            for (vvv = p2->xy_time.begin(); vvv < p2->xy_time.end(); ++vvv, ++vv)
            {
              if (ros::Time::now().toSec() - *vvv > WINDOW_TIME)
              {
                p2->xy_time.erase(vvv);
                p2->xy.erase(vv);
                break;
              }
            }
            (p2->xy).resize((p2->xy).size());
            (p2->xy_time).resize((p2->xy_time).size());

            int z;
            for (z = 0; z < (p2->xy).size() - 1; z++)
            {
              if ((p2->id) % 7 == 0)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkRed, 5));
              }
              else if ((p2->id) % 7 == 1)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkBlue, 5));
              }
              else if ((p2->id) % 7 == 2)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkGreen, 5));
              }
              else if ((p2->id) % 7 == 3)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkCyan, 5));
              }
              else if ((p2->id) % 7 == 4)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkGray, 5));
              }
              else if ((p2->id) % 7 == 5)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkMagenta, 5));
              }
              else if ((p2->id) % 7 == 6)
              {
                drawLine((p2->xy)[z].x, (p2->xy)[z].y, (p2->xy)[z + 1].x, (p2->xy)[z + 1].y, QPen(Qt::darkYellow, 5));
              }
            }
            if ((p2->id) % 7 == 0)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::red, 15));
            }
            else if ((p2->id) % 7 == 1)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::blue, 15));
            }
            else if ((p2->id) % 7 == 2)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::green, 15));
            }
            else if ((p2->id) % 7 == 3)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::cyan, 15));
            }
            else if ((p2->id) % 7 == 4)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::gray, 15));
            }
            else if ((p2->id) % 7 == 5)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::magenta, 15));
            }
            else if ((p2->id) % 7 == 6)
            {
              drawPoint((p2->xy)[z].x, (p2->xy)[z].y, QPen(Qt::yellow, 15));
            }
            // Time management
            break;
          }
          else if (p2 == tracker_info_array2.end() - 1)
          {
            // candidate not on the array
            tmp_tracker_info2.id = msgRawdata.tracking_grid[i].id;
            tmp_tracker_info2.flag = 1;
            tmp_tracker_info2.count = 0;
            tracker_point tmp_xy;
            tmp_xy.x = msgRawdata.tracking_grid[i].x;
            tmp_xy.y = msgRawdata.tracking_grid[i].y;
            tmp_tracker_info2.xy.resize(0);
            tmp_tracker_info2.xy.clear();
            tmp_tracker_info2.xy.push_back(tmp_xy);
            tmp_tracker_info2.start_time = ros::Time::now().toSec();
            tmp_tracker_info2.xy_time.resize(0);
            tmp_tracker_info2.xy_time.clear();
            tmp_tracker_info2.xy_time.push_back(ros::Time::now().toSec());
            tracker_info_array2.insert(tracker_info_array2.begin(), tmp_tracker_info2);
            break;
          }
        }
      }
      else
      {
        // no array
        tmp_tracker_info2.id = msgRawdata.tracking_grid[i].id;
        tmp_tracker_info2.flag = 1;
        tmp_tracker_info2.count = 0;
        tracker_point tmp_xy;
        tmp_xy.x = msgRawdata.tracking_grid[i].x;
        tmp_xy.y = msgRawdata.tracking_grid[i].y;
        tmp_tracker_info2.xy.resize(0);
        tmp_tracker_info2.xy.clear();
        tmp_tracker_info2.xy.push_back(tmp_xy);
        tmp_tracker_info2.start_time = ros::Time::now().toSec();
        tmp_tracker_info2.xy_time.resize(0);
        tmp_tracker_info2.xy_time.clear();
        tmp_tracker_info2.xy_time.push_back(ros::Time::now().toSec());
        tracker_info_array2.push_back(tmp_tracker_info2);
      }
    }
  }
  if (tracker_info_array2.size() > 0)
  {
    tracker_info_array2.resize(tracker_info_array2.size());
  L:
    int n_flag = 0;
    for (p2 = tracker_info_array2.begin(); p2 != tracker_info_array2.end(); ++p2)
    {
      if (p2->flag == 0)
      {
        p2->xy.resize(0);
        p2->xy.clear();
        p2->xy_time.resize(0);
        p2->xy.resize(0);
        tracker_info_array2.erase(p2);
        n_flag = 1;
        break;
      }
    }
    if (n_flag == 1)
    {
      goto L;
    }
  }
  /*
  system("clear");
  if(tracker_info_array2.size())
  {
      std::cout <<
  "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

  for(std::vector<tracker_param2>::iterator vw = tracker_info_array2.begin(); vw < tracker_info_array2.end(); ++vw)
    {
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "id "   << vw->id   << std::endl;
    std::cout << "flag " << vw->flag << std::endl;
    std::cout << "count "<< vw->count << std::endl;
    std::cout << "xy"    << std::endl;
      for(int i=0;i<vw->xy.size();i++)
      {
      std::cout << "x "  << vw->xy[i].x << " y "  << vw->xy[i].y << std::endl;
      }
      std::cout << "start_time "<< vw->start_time << std::endl;
      for(int i=0;i<vw->xy_time.size();i++)
      {
      std::cout << "each time "  << vw->xy_time[i] << std::endl;
      }
      std::cout << "---------------------------------------------------------" << std::endl;
      double ExeTime = ros::Time::now().toSec() - vw->start_time;
    }
   std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
  << std::endl;
  }
  */
  update();
}

void Viewer::setPortable(int x, int y)
{
  QLabel *my_label = new QLabel(this);
  QPixmap photo("./src/ros_tms/tms_dev/portable_viewer/image/portable.png");
  my_label->setGeometry(x - 14, y - 23, 28, 46);
  // my_label->setStyleSheet("background-color: red;");
  my_label->setStyleSheet("background-color: rgba(255, 255, 255, 0);");
  my_label->setPixmap(photo);
  my_label->show();
}
