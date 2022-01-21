#ifndef CHANGE_H
#define CHANGE_H
#include "utility.h"
#include "constraints.h"

class Change {
  public:
    Change() {}
    Change(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();

    ramp_msgs::Path path_;

  private:
    Constraints checkConstraints_;
    Utility utility_;
};

#endif
