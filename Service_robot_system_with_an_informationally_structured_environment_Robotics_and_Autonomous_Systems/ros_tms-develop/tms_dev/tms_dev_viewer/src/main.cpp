//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : main function
// @author : Yoonseok Pyo, Masahide Tanaka
// @version: Ver0.9.5 (since 2012.05.17)
// @date   : 2012.11.14
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui/QApplication>

#include <ros/ros.h>

#include "mainwindow.h"
#include "qnode.hpp"

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  QNode qnode(argc, argv, "tms_viewer");
  MainWindow window(&qnode);
  window.show();
  return app.exec();
}

//------------------------------------------------------------------------------
