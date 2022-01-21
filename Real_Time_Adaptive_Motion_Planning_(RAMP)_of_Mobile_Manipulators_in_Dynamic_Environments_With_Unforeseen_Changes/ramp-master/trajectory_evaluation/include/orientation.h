#ifndef ORIENTATION_H
#define ORIENTATION_H
#include "utility.h"
#include "ramp_msgs/RampTrajectory.h"

class Orientation {
  public:
    Orientation();

    const double perform(const ramp_msgs::RampTrajectory& trj);
    const double getDeltaTheta(const ramp_msgs::RampTrajectory& trj) const;
    const double getPenalty(const ramp_msgs::RampTrajectory& trj) const;

    double currentTheta_;
    double theta_at_cc_;
    ramp_msgs::RampTrajectory trajectory_;

    double Q_;
    
    Utility utility_;
};

#endif
