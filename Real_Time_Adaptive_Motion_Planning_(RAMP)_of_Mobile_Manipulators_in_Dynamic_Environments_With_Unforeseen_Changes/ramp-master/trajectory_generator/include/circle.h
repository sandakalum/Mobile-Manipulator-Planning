#ifndef CIRCLE_H
#define CIRCLE_H
#include "utility.h"

#define CYCLE_TIME_IN_SECONDS 0.1

class Circle {
public:
  
  Circle();
  ~Circle();

  const std::vector<ramp_msgs::MotionState>   generatePoints();
  void init(const ramp_msgs::MotionState s);
private:
  ReflexxesData reflexxesData_;
  ros::Duration timeFromStart_;
  ros::Duration timeCutoff_;
  Utility utility_;

  void initReflexxes();

  const ramp_msgs::MotionState buildMotionState(const ReflexxesData data);
  
  // Initialize variables just after receiving a service request
  void setReflexxesCurrent();
  void setReflexxesTarget();
  void setReflexxesSelection();
  
  const ramp_msgs::MotionState spinOnce();

  // Returns true if the target has been reached
  const bool finalStateReached();
  
  ramp_msgs::MotionState start_;
  ramp_msgs::MotionState center_;
  double r_, v_, w_;
  double t;
  double initCircleTheta_;
};

#endif
