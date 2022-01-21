#include <QtGui>
#include <QPainter>

#include <ros/ros.h>

#include "viewer.h"
#include "qnode.hpp"
#include "iostream"
#include "std_msgs/String.h"
#include <tms_msg_ss/pot_sensor.h>

Viewer::Viewer(QNode *node, QWidget *parent) :
     QWidget(parent),
     qnode(node)
    ,viewer_count(0)
    ,viewer_distance("")
    ,viewer_people_sense("")
    ,viewer_flame("")
    ,viewer_mike("")
    ,viewer_gas("")
    ,viewer_temperature("")
    ,viewer_humidity("")
    ,viewer_lrf_key("")
    ,all_info("")
{
    ros::init(qnode->init_argc,qnode->init_argv,qnode->node_name);

    ros::NodeHandle nh;
    subscriber = nh.subscribe("odroid_info", 10, &Viewer::Callback, this);

    QTimer* timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( view() ) );
    timer->start( 1 );
}

Viewer::~Viewer()
{

}

void Viewer::view()
{
    ros::spinOnce();
}

void Viewer::Callback(const tms_msg_ss::pot_sensor::ConstPtr& msg)
{
    viewer_count         = msg->count;
    viewer_distance      = msg->distance.c_str();
    viewer_people_sense  = msg->motion.c_str();
    viewer_flame         = msg->flame.c_str();
    viewer_mike          = msg->microphone.c_str();
    viewer_gas           = msg->gas.c_str();
    viewer_temperature   = msg->temperature.c_str();
    viewer_humidity      = msg->humidity.c_str();
    viewer_lrf_key       = msg->lrf_key.c_str();

    sprintf(temp,"count:             %d" ,viewer_count);
    strcat(temp,"\ntemperature:  ");
    strcat(temp,viewer_temperature);
    strcat(temp,"\nhumidity:         ");
    strcat(temp,viewer_humidity);
    strcat(temp,"\nmotion:            ");
    strcat(temp,viewer_people_sense);
    strcat(temp,"\nflame:               ");
    strcat(temp,viewer_flame);
    strcat(temp,"\nmicrophone:  ");
    strcat(temp,viewer_mike);
    strcat(temp,"\ngas:                    ");
    strcat(temp,viewer_gas);
    strcat(temp,"\nlrf:                      ");
    strcat(temp,viewer_lrf_key);
    all_info = temp;
    test_count++;
}
