//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : main function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------

#include <QApplication>
#include <ros/ros.h>

#include "mainwindow.h"
#include "iostream"
#include "qnode.hpp"

int main(int argc, char **argv)
{
  std::cout << "-----start viewer-----" << std::endl;
  QApplication app(argc, argv);  // Qapplicatio statement
  QNode qnode(argc, argv, "test_viewer");
  MainWindow window(&qnode);
  window.show();
  return app.exec();
}
