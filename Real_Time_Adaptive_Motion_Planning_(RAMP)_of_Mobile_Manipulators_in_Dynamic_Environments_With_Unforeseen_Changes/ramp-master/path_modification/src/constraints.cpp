#include "constraints.h"


const bool Constraints::validKPForPath(const ramp_msgs::KnotPoint kp, const ramp_msgs::Path p) const
{
  if(kp.motionState.positions.size() == 0)
  {
    return false;
  }

  double L = 0.25;

  for(uint8_t i=0;i<p.points.size();i++)
  {
    if( sqrt( pow(p.points.at(i).motionState.positions.at(0) - kp.motionState.positions.at(0), 2) +
              pow(p.points.at(i).motionState.positions.at(1) - kp.motionState.positions.at(1), 2) ) < L)
    {
      return false;
    }
  }

  return true;
}
