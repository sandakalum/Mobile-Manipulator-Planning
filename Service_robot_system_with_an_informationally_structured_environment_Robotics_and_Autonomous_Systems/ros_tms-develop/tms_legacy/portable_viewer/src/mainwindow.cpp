//------------------------------------------------------------------------------
// @file   : mainwindow.cpp
// @brief  : window function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------
#include <QString>

#include "mainwindow.h"
#include <iostream>
#include <string>

MainWindow::MainWindow(QNode *node, QWidget *parent) : QMainWindow(parent), qnode(node), window_count(0)
{
  setupUi(this);
  setWindowTitle("test_viewer");
  this->setStyleSheet("background-color: rgb(238,233,233);");

  // photo setting
  viewer = new Viewer(node, parent);
  this->setCentralWidget(viewer);

  // Image_viewer
  connect(this->CLEAR, SIGNAL(clicked()), this, SLOT(clear_Image()));
  connect(this->RELOAD, SIGNAL(clicked()), this, SLOT(reload_Image()));

  // track_viewer
  connect(this->Nothing, SIGNAL(clicked()), this, SLOT(track_Nothing()));
  connect(this->Gradual_disappear, SIGNAL(clicked()), this, SLOT(track_Gradual_disappear()));
  connect(this->All_remind, SIGNAL(clicked()), this, SLOT(track_All_remind()));
  connect(this->Grid_on, SIGNAL(clicked()), this, SLOT(grid_On()));
  connect(this->Grid_off, SIGNAL(clicked()), this, SLOT(grid_Off()));

  // laser_viewer
  connect(this->Laser_on, SIGNAL(clicked()), this, SLOT(laser_on()));
  connect(this->Laser_off, SIGNAL(clicked()), this, SLOT(laser_off()));

  QTimer *timer;
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(viewData()));
  timer->start(1);
}

MainWindow::~MainWindow()
{
}

void MainWindow::viewData()
{
}

void MainWindow::clear_Image()
{
  viewer->clearImage_Button = true;
}

void MainWindow::reload_Image()
{
  viewer->reloadImage_Button = true;
}

void MainWindow::track_Nothing()
{
  viewer->track_Nothing_button = true;
  viewer->track_Gradual_disappear = false;
  viewer->track_All_remind_button = false;
}

void MainWindow::track_Gradual_disappear()
{
  viewer->track_Gradual_disappear = true;
  viewer->track_All_remind_button = false;
  viewer->track_Nothing_button = false;
}

void MainWindow::track_All_remind()
{
  viewer->track_All_remind_button = true;
  viewer->track_Gradual_disappear = false;
  viewer->track_Nothing_button = false;
}

void MainWindow::grid_On()
{
  viewer->grid_on = true;
  viewer->grid_off = false;
}

void MainWindow::grid_Off()
{
  viewer->grid_off = true;
  viewer->grid_on = false;
}

void MainWindow::laser_on()
{
  viewer->laser_on = true;
  viewer->laser_off = false;
}

void MainWindow::laser_off()
{
  viewer->laser_on = false;
  viewer->laser_off = true;
}
