#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//------------------------------------------------------------------------------
#include <QMainWindow>

#include "ui_mainwindow.h"
#include "viewer.h"
#include "qnode.hpp"

class Viewer;

namespace Ui {
class MainWindow;
}

//------------------------------------------------------------------------------
class MainWindow : public QMainWindow, private Ui_MainWindow
{
    Q_OBJECT
    
public:
    MainWindow(QNode *node, QWidget *parent = 0);
    ~MainWindow();

private Q_SLOTS:
    void viewData();


private:
    Viewer *viewer;
    QNode  *qnode;
};

//------------------------------------------------------------------------------
#endif // MAINWINDOW_H

//------------------------------------------------------------------------------
