#include "swap.h"


Swap::Swap(const ramp_msgs::Path p) : path_(p) {}


/** Swap does NOT swap the velocity values! Only the positions! */
const ramp_msgs::Path Swap::perform() 
{

  if(path_.points.size() > 3) 
  {
    unsigned int i_knotPoint1 = rand() % (path_.points.size()-2) + 1;
    unsigned int i_knotPoint2; 
    
    // Make sure the two configurations are different
    do 
    {
      i_knotPoint2 = rand() % (path_.points.size()-2) + 1;
    } while(i_knotPoint1 == i_knotPoint2);

    //std::cout<<"\ni_knotPoint1: "<<i_knotPoint1<<" i_knotPoint2: "<<i_knotPoint2;
    
    // Swap the knot points' positions
    std::vector<double> temp = path_.points.at(i_knotPoint1).motionState.positions;
    path_.points.at(i_knotPoint1).motionState.positions = path_.points.at(i_knotPoint2).motionState.positions;
    path_.points.at(i_knotPoint2).motionState.positions = temp;

  }

  return path_;
}
