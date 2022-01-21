#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/TrajectoryResponse.h"
#include "ramp_msgs/Range.h"


#define PI 3.14159f
#define CYCLE_TIME_IN_SECONDS 0.1

class Utility {
  public:
    
    Utility();
    ~Utility() {}

    std::vector<ramp_msgs::Range> standardRanges;
    
    const double planarDistance(const std::vector<double> a, const std::vector<double> b) const;

    const double findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const;
    const double findAngleFromAToB(const trajectory_msgs::JointTrajectoryPoint a, const trajectory_msgs::JointTrajectoryPoint b) const;
    
    const double findDistanceBetweenAngles(const double a1, const double a2) const;
    
    const double displaceAngle(const double a1, double a2) const;
    
    const double getEuclideanDist(const std::vector<double> a, std::vector<double> b) const;
    
    const std::string toString(const trajectory_msgs::JointTrajectoryPoint p) const;
    const std::string toString(const ramp_msgs::RampTrajectory traj) const;
    const std::string toString(const ramp_msgs::Path p) const;
    const std::string toString(const ramp_msgs::MotionState c) const;
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
};
#endif
