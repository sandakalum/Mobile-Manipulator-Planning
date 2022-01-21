#ifndef ROS_H
#define ROS_H

#include <QThread>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ramp_msgs/Population.h"

#include <QGraphicsScene>


class Ros : public QThread {

    Q_OBJECT

private:
    void subscribe();

    bool initialized; // tell if the ros thread has been initlized
    ros::Subscriber popSub; // population topic subscriber
    ros::Subscriber popSub_1; // population topic subscriber
    ros::Subscriber popSub_2; // population topic subscriber


signals :
    void population(const ramp_msgs::Population&); // Whole population of trajectories

public slots:

public:
    ~Ros();
    Ros();

	void run();
    void init(int argc, char *argv[]);
    void populationCallback(const ramp_msgs::Population& msg); // Callback of the topic population. We expect the first trajectory to be the best one
};


#endif // ROS_H
