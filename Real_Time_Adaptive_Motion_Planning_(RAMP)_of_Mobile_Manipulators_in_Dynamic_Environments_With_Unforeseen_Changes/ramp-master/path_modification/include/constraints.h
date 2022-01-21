#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H
#include "utility.h"


class Constraints
{
  public:
    Constraints() {}

    const bool validKPForPath(const ramp_msgs::KnotPoint kp, const ramp_msgs::Path p) const;
};

#endif
