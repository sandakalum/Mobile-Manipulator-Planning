#ifndef VIEWER_H
#define VIEWER_H

//------------------------------------------------------------------------------
#include <QWidget>
#include <QtGui>

#include <ros/ros.h>

#include "mainwindow.h"
#include "qnode.hpp"
#include <tms_msg_ss/pot_sensor.h>

//------------------------------------------------------------------------------
class Viewer: public QWidget
{
    Q_OBJECT

public:
    int viewer_count;
    int test_count;
    char temp[250];
    const char  *viewer_distance;
    const char  *viewer_people_sense;
    const char  *viewer_flame;
    const char  *viewer_mike;
    const char  *viewer_gas;
    const char  *viewer_temperature;
    const char  *viewer_humidity;
    const char  *viewer_lrf_key;

    const char *all_info;

    Viewer(QNode *node, QWidget *parent = 0);
    virtual ~Viewer();

public Q_SLOTS:
    void view();

private:
    QNode *qnode;
    ros::Subscriber subscriber;
    void Callback(const tms_msg_ss::pot_sensor::ConstPtr& msg);
};

//------------------------------------------------------------------------------
#endif // VIEWER_H
//------------------------------------------------------------------------------
