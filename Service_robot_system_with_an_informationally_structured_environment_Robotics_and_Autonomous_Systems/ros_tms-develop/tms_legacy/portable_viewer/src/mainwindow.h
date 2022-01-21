//------------------------------------------------------------------------------
// @file   : mainwindow.h
// @brief  : window function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#ifndef Q_MOC_RUN

//------------------------------------------------------------------------------
#include <QMainWindow>

#include "ui_mainwindow.h"
#include "viewer.h"
#include "qnode.hpp"

#include <iostream>
#include <string>

class Viewer;

namespace Ui
{
class MainWindow;
}

//------------------------------------------------------------------------------
class MainWindow : public QMainWindow, private Ui_MainWindow
{
  Q_OBJECT

public:
  MainWindow(QNode *node, QWidget *parent = 0);
  ~MainWindow();

  int window_count;

private Q_SLOTS:
  void viewData();
  void clear_Image();
  void reload_Image();
  void track_Nothing();
  void track_Gradual_disappear();
  void track_All_remind();
  void grid_On();
  void grid_Off();
  void laser_on();
  void laser_off();

private:
  Viewer *viewer;
  QNode *qnode;
};

#endif

//------------------------------------------------------------------------------
#endif  // MAINWINDOW_H

//------------------------------------------------------------------------------
