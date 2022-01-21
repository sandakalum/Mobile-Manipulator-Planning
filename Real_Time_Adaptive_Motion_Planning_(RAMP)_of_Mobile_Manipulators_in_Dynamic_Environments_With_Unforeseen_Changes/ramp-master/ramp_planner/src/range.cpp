#include "range.h"

Range::Range() {}

Range::Range(const float min, const float max) {
  msg_.min = min;
  msg_.max = max;
}

Range::Range(const ramp_msgs::Range r) {
  msg_.min = r.min;
  msg_.max = r.max;
}

Range::~Range() {}


const float Range::random() const {
  if(msg_.min == 0 && msg_.max == 0)
    return 0;

  return ( msg_.min + (float)rand() / ((float)RAND_MAX / (msg_.max - msg_.min)) ); 
}



const std::string Range::toString() const {
  std::ostringstream result;

  result<<"\nmsg_.min: "<<msg_.min<<" msg_.max: "<<msg_.max;

  return result.str();
}
