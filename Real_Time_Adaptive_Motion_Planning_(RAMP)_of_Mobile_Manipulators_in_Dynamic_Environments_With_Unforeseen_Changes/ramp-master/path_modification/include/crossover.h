#ifndef CROSSOVER_H
#define CROSSOVER_H
#include "ramp_msgs/Path.h"

class Crossover {
  public:
    Crossover() {}
    Crossover(const ramp_msgs::Path p1, const ramp_msgs::Path p2);

    const std::vector<ramp_msgs::Path> perform();

    ramp_msgs::Path path1_;
    ramp_msgs::Path path2_;
  private:

};

#endif
