#include "mainwindow.h"
#include "ui_mainwindow.h"

#ifdef Q_WS_WIN
#include "windows.h"
#include "mmsystem.h"
#endif


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //set the parent widgets to the different widget created
    r.setParent(this);
    ui->setupUi(this);
    trajView.setParent(ui->frame);

    // Connect the ros thread and the trajView
    connect(&r,SIGNAL(population(const ramp_msgs::Population&)),&trajView, SLOT(population(const ramp_msgs::Population&)));

    r.init(0, NULL); //initialize the ros thread
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::resizeEvent(QResizeEvent *){//execute this function when the window size changes
    trajView.size_changed();
}
