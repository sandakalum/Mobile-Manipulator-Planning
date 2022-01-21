#ifndef MOVE_H
#define MOVE_H
#include "utility.h"
#include "constraints.h"
#include "range.h"

class Move {
  public:
    Move() {}
    Move(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();

    ramp_msgs::Path path_;
    double dir_;
    double dist_;

  private:
    Constraints checkConstraints_;
    Utility utility_;
};

#endif
