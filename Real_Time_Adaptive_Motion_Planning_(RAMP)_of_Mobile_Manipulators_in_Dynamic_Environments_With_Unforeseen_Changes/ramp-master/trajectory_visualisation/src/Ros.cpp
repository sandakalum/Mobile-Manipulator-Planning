#include "Ros.h"
#include <QImage>
#include <QGraphicsTextItem>
#include <QPainter>
#include <QGraphicsView>
#include <math.h>
#include <QImageReader>

#include <vector>

using namespace std;

Ros::Ros(){
    initialized = false;
}


Ros::~Ros(){

}

void Ros::subscribe()
//function that initialize every ros variables declared

{
    ros::NodeHandle n;

    //Subscribe to topics
    popSub= n.subscribe("population",1000, &Ros::populationCallback,this);
    popSub_1= n.subscribe("/robot_0/population",1000, &Ros::populationCallback,this);
    popSub_2= n.subscribe("/robot_1/population",1000, &Ros::populationCallback,this);

    initialized = true;
}

void Ros::init(int argc, char *argv[]){
    ros::init(argc, argv,"trajectory_view");

    // To be able to send a signal with ramp_msgs::Population, we first need to register the type
    qRegisterMetaType<ramp_msgs::Population>("ramp_msgs::Population");

    this->subscribe();
    this->start();
}

void Ros::run(){
    ros::spin();
}

void Ros::populationCallback(const ramp_msgs::Population& msg)
{
    Q_EMIT population(msg);
}
