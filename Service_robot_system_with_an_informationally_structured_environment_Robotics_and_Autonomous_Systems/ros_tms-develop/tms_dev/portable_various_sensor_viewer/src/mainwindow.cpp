#include <QString>

#include "mainwindow.h"

MainWindow::MainWindow(QNode *node, QWidget *parent) :
    QMainWindow(parent),
    qnode(node)
{
  setupUi(this);
  setWindowTitle("portable_viewer");
  this->setStyleSheet("background-color: rgb(205,201,201);");

  viewer = new Viewer(node, parent);
  QPixmap pixmap("/home/kurt/portable.png");
  this->photo->setPixmap(pixmap);
  this->photo->setMask(pixmap.mask());

  this->window1->setStyleSheet("background-color : rgb(211,211,211);");
  this->window2->setStyleSheet("background-color : rgb(211,211,211);");

  QTimer* timer;
  timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( viewData() ) );
  timer->start( 50 );
}

MainWindow::~MainWindow()
{

}

void MainWindow::viewData()
{
  this->window1->setText(QString(viewer->all_info));
  //this->window2->setNum(viewer->test_count);
}
