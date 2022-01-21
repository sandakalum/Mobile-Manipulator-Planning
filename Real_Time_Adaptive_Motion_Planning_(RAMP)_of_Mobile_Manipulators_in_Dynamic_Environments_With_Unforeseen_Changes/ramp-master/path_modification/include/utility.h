#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <sstream>
#include <math.h>
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Range.h"
#include <ros/console.h>

#define PI 3.14159f

class Utility {
  public:
    Utility();

    std::vector<ramp_msgs::Range> standardRanges_;

    const double positionDistance(const std::vector<double> a, const std::vector<double> b) const;
    
    const double findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const;

    const double findDistanceBetweenAngles(const double a1, const double a2) const;
    
    const double displaceAngle(const double a1, double a2) const;

    const std::string toString(const ramp_msgs::Path p) const;    
};
#endif
