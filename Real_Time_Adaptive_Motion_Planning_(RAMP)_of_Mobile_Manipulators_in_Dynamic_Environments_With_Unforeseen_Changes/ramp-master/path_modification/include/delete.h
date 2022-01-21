#ifndef DELETE_H
#define DELETE_H
#include "ramp_msgs/Path.h"

class Delete {
  public:
    Delete() {}
    Delete(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();
    
    ramp_msgs::Path path_;
  
  private:

};

#endif 
