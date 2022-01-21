#ifndef SWAP_H
#define SWAP_H
#include "ramp_msgs/Path.h"

class Swap {
  public:
    Swap() {} 
    Swap(const ramp_msgs::Path p);
   
    const ramp_msgs::Path perform();
   
    ramp_msgs::Path path_; 
};

#endif
