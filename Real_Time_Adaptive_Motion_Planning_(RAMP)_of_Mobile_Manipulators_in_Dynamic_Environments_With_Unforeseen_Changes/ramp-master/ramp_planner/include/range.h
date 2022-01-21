#ifndef RANGE_H
#define RANGE_H
#include "ramp_msgs/Range.h"

class Range {
  public:
    Range();
    Range(const float min, const float max);
    Range(const ramp_msgs::Range r);
    ~Range();

    ramp_msgs::Range msg_;

    /** This method returns a random value in the range */
    const float random() const;
    const std::string toString() const;

};

#endif
