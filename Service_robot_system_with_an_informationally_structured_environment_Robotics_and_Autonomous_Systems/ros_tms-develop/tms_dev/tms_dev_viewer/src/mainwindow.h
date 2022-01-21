#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//------------------------------------------------------------------------------
#include <QMainWindow>

#include "ui_mainwindow.h"
#include "viewer.h"
#include "qnode.hpp"

//------------------------------------------------------------------------------
class Viewer;

//------------------------------------------------------------------------------
namespace Ui
{
class MainWindow;
}

//------------------------------------------------------------------------------
class MainWindow : public QMainWindow, private Ui_MainWindow
{
  Q_OBJECT

public:
  QAbstractItemModel *table_model;

  MainWindow(QNode *node, QWidget *parent = 0);
  ~MainWindow();

  void readSettings();   // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);

private Q_SLOTS:
  void zoomEvent(const QString &scale);
  void zoomInEvent();
  void zoomOutEvent();
  void captureEvent();
  void pressdIndex(const QModelIndex &index);
  void viewData();
  void eventChangeOption();
  void trajectoryOption();
  void processIntensityViewValue();

private:
  QTextCodec *codec;
  QItemSelectionModel *table_selectionModel;
  QComboBox *imageScaleCombo;
  Viewer *viewer;
  QNode *qnode;
};

//------------------------------------------------------------------------------
#endif  // MAINWINDOW_H

//------------------------------------------------------------------------------
