#ifndef CONTROL_HANDLER_H
#define CONTROL_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Population.h"
#include "std_msgs/Bool.h"

class ControlHandler {
  public:
    ControlHandler(const ros::NodeHandle& h);

    void send(ramp_msgs::RampTrajectory bestTrajec);
    void sendPopulation(ramp_msgs::Population population);
    void sendIC(std_msgs::Bool value);
    void sendObIC(const int i, std_msgs::Bool value);

  private:
    ros::NodeHandle handle_;
    ros::Publisher pub_bestTrajec_;
    ros::Publisher pub_population_;
    ros::Publisher pub_imminent_collision_;
    std::vector<ros::Publisher> pub_ob_imminent_collision_;
};

#endif
