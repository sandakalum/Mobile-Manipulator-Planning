#include "utility.h"

Utility::Utility() {
  for(unsigned int i=0;i<3;i++) {
    ramp_msgs::Range temp;
    temp.min = (i < 2 ? 0   : -PI);
    temp.max = (i < 2 ? 2 :  PI);
    standardRanges_.push_back(temp);
  }
}


/** This method returns the Euclidean distance between two position vectors */
const double Utility::positionDistance(const std::vector<double> a, const std::vector<double> b) const 
{
  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} // End euclideanDistance


/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const double Utility::findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const 
{
  double result;

  // If the positions are the same, return the orientation the robot already has
  if(fabs(positionDistance(a, b)) < 0.01 && a.size() > 2)
  {
    return a.at(2);
  }

  // Find the distances in x,y directions and Euclidean distance
  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);

  // Fails when vector from a to be is in 3rd quadrant
  //result = atan(d_y / d_x);
  
  double euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );
  // If the positions are the same,
  // Set the result to the starting orientation if one is provided
  // Or to 0 if no starting orientation is provided
  if(euc_dist <= 0.0001) 
  {
    result = 0;
  }

  // If b is in the 1st or 2nd quadrants
  else if(d_y > 0) 
  {
    result = acos(d_x / euc_dist);
  }

  // If b is in the 3rd quadrant, d_y<0 & d_x<0
  else if(d_x < 0) 
  {
    result = -PI - asin(d_y / euc_dist);
  }

  // If b is in the 4th quadrant, d_y<=0 & d_x>=0
  else 
  {
    result = asin(d_y / euc_dist); 
  }

  return result;
} // End findAngleFromAToB


/** This method returns distance between orientations a1 and a2. The distance is in the range [-PI, PI]. */
const double Utility::findDistanceBetweenAngles(const double a1, const double a2) const 
{
  double result;
  double difference = a2 - a1;
  
  // If difference > pi, the result should be in [-PI,0] range
  if(difference > PI) 
  {
    difference = fmodf(difference, PI);
    result = difference - PI;
  }

  // If difference < -pi, the result should be in [0,PI] range
  else if(difference < -PI) 
  {
    result = difference + (2*PI);
  }

  // Else, the difference is fine
  else 
  {
    result = difference;
  }

  return result;
} // End findDistanceBetweenAngles



const double Utility::displaceAngle(const double a1, double a2) const {

  a2 = fmodf(a2, 2*PI);

  if(a2 > PI) 
  {
    a2 = fmodf(a2,PI) - PI;
  }

  return findDistanceBetweenAngles(-a1, a2);
} // End displaceAngle




const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath:";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n  "<<i<<": (";

    result<<path.points.at(i).motionState.positions.at(0);
    for(unsigned int k=1;k<path.points.at(i).motionState.positions.size();k++) {
      result<<", "<<path.points.at(i).motionState.positions.at(k);
    }
    result<<")";
  }

  return result.str();
}

