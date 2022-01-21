#ifndef LINE_H
#define LINE_H
#include "utility.h"

#define CYCLE_TIME_IN_SECONDS 0.1

class Line {
public:
  
  Line();
  ~Line();

  const std::vector<ramp_msgs::MotionState>   generatePoints();
  void init(const ramp_msgs::MotionState start, 
            const ramp_msgs::MotionState goal);

private:

  ReflexxesData reflexxesData_;
  ramp_msgs::MotionState start_, goal_;
  ros::Duration timeFromStart_;
  ros::Duration timeCutoff_;
  Utility utility_;

  const ramp_msgs::MotionState buildMotionState(const ReflexxesData data);

  void initReflexxes();

  // Initialize variables just after receiving a service request
  void setReflexxesCurrent();
  void setReflexxesTarget();
  void setReflexxesSelection();
  
  const ramp_msgs::MotionState spinOnce();

  // Returns true if the target has been reached
  const bool finalStateReached();
  
};

#endif
