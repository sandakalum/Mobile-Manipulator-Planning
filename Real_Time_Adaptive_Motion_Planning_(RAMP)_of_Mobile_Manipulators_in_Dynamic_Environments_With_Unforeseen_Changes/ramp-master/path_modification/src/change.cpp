#include "change.h"



Change::Change(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Change::perform() 
{
  //ROS_INFO("Before: %s", utility_.toString(path_).c_str()); 

  if(path_.points.size() > 2) 
  {

    // Randomly choose a knot point to change
    unsigned int i_knotPoint = rand() % (path_.points.size()-2) + 1;
    
    // Clear the knot point's positions
    path_.points.at(i_knotPoint).motionState.positions.clear();


    ramp_msgs::KnotPoint kp;
    ramp_msgs::Path tempPath = path_;
    tempPath.points.erase(tempPath.points.begin()+i_knotPoint);
    
    while(!checkConstraints_.validKPForPath(kp, tempPath))
    {
      // Generate new, random values for the positions
      kp.motionState.positions.clear();
      for(unsigned int i=0;i<path_.points.at(0).motionState.positions.size();i++) 
      {
        
        // Generate a random value for each K in the specified range
        double  min = utility_.standardRanges_.at(i).min;
        double  max = utility_.standardRanges_.at(i).max;
        float temp = (min == 0 && max == 0) ? 0 :      
              ( min + (float)rand() / ((float)RAND_MAX / (max - min)) );

        // Push the new value onto positions
        kp.motionState.positions.push_back(temp);
      } // end for
    } // end while

    path_.points.at(i_knotPoint) = kp;
  } // end if points.size()>2

  //ROS_INFO("After: %s", utility_.toString(path_).c_str()); 
  return path_;
}
