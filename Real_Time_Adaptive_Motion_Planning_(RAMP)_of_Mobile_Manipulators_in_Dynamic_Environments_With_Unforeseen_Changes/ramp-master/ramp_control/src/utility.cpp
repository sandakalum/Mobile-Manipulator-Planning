#include "utility.h"

Utility::Utility() {}


/** This method returns the Euclidean distance between two position vectors */
const double Utility::planarDistance(const std::vector<double> a, const std::vector<double> b) const {

  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} //End euclideanDistance



const double Utility::findAngleFromAToB(const trajectory_msgs::JointTrajectoryPoint a, const trajectory_msgs::JointTrajectoryPoint b) const {
  std::vector<double> c;
  c.push_back(a.positions.at(0));
  c.push_back(a.positions.at(1));
  
  std::vector<double> d;
  d.push_back(b.positions.at(0));
  d.push_back(b.positions.at(1));

  return findAngleFromAToB(c, d);
}



/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const double Utility::findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const {
  double result;

  //std::cout<<"\na: ["<<a.at(0)<<", "<<a.at(1)<<"] b: ["<<b.at(0)<<", "<<b.at(1)<<"]";

  // Find the distances in x,y directions and Euclidean distance
  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  double euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );
  
  // If the positions are the same,
  // Set the result to the starting orientation if one is provided
  // Or to 0 if no starting orientation is provided
  // TODO: Make the comparison proportionate to size of space
  if(euc_dist <= 0.000001) {
    result = 0;
  }

  // If b is in the 1st or 2nd quadrants
  else if(d_y > 0) {
    result = acos(d_x / euc_dist);
  }

  // If b is in the 3rd quadrant, d_y<0 & d_x<0
  else if(d_x < 0) {
    result = -PI - asin(d_y / euc_dist);
  }

  // If b is in the 4th quadrant, d_y<=0 & d_x>=0
  else {
    result = asin(d_y / euc_dist); 
  }

  //std::cout<<"\nresult: "<<result;
  return result;
} //End findAngleFromAToB


/** This method returns distance between orientations a1 and a2. The distance is in the range [-PI, PI]. */
const double Utility::findDistanceBetweenAngles(const double a1, const double a2) const {
  double result;
  double difference = a2 - a1;
  
  
  // If difference > pi, the result should be in [-PI,0] range
  if(difference > PI) {
    difference = fmodf(difference, PI);
    result = difference - PI;
  }

  // If difference < -pi, the result should be in [0,PI] range
  else if(difference < -PI) {
    result = difference + (2*PI);
  }

  // Else, the difference is fine
  else {
    result = difference;
  }

  return result;
} //End findDistanceBetweenAngles



const double Utility::displaceAngle(const double a1, double a2) const {

  a2 = fmodf(a2, 2*PI);

  if(a2 > PI) {
    a2 = fmodf(a2,PI) - PI;
  }

  return findDistanceBetweenAngles(-a1, a2);
} //End displaceAngle



/** a and b must be the same size */
const double Utility::getEuclideanDist(const std::vector<double> a, const std::vector<double> b) const {
  double result=0;

  for(unsigned int i=0;i<a.size();i++) {
    result += pow(a.at(i) - b.at(i), 2);
  }
  
  result = sqrt(result);
  return result;
}


const std::string Utility::toString(const ramp_msgs::MotionState mp) const {
  std::ostringstream result;

  result<<"\np: [ ";
  for(unsigned int i=0;i<mp.positions.size();i++) {
    result<<mp.positions.at(i)<<" ";
  }
  result<<"]";

  result<<"\nv: [ ";
  for(unsigned int i=0;i<mp.velocities.size();i++) {
    result<<mp.velocities.at(i)<<" ";
  }
  result<<"]";

  result<<"\na: [ ";
  for(unsigned int i=0;i<mp.accelerations.size();i++) {
    result<<mp.accelerations.at(i)<<" ";
  }
  result<<"]";

  result<<"\nj: [ ";
  for(unsigned int i=0;i<mp.jerks.size();i++) {
    result<<mp.jerks.at(i)<<" ";
  }
  result<<"]";

  result<<"\n time: "<<mp.time;

  return result.str();
}

const std::string Utility::toString(const ramp_msgs::KnotPoint kp) const {
  std::ostringstream result;

  result<<"\nMotion State: "<<toString(kp.motionState);
  result<<", Stop time: "<<kp.stopTime;

  return result.str();
}




const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath: ";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n "<<i<<": "<<toString(path.points.at(i));
  }

  return result.str();
}



const std::string Utility::toString(const trajectory_msgs::JointTrajectoryPoint p) const {
  std::ostringstream result;

  //Positions
  result<<"\n       Positions: ("<<p.positions.at(0);
  for(unsigned int k=1;k<p.positions.size();k++) {
    result<<", "<<p.positions.at(k);
  }
  result<<")";

  //Velocities
  result<<"\n       Velocities: ("<<p.velocities.at(0);
  for(unsigned int k=1;k<p.velocities.size();k++) {
    result<<", "<<p.velocities.at(k);
  }
  result<<")";
  
  //Accelerations
  result<<"\n       Accelerations: ("<<p.accelerations.at(0);
  for(unsigned int k=1;k<p.accelerations.size();k++) {
    result<<", "<<p.accelerations.at(k);
  }
  result<<")";
  
  result<<"\n Time From Start: "<<p.time_from_start;

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::RampTrajectory traj) const {
  std::ostringstream result;

  result<<"\n Knot Points:";

  for(unsigned int i=0;i<traj.i_knotPoints.size();i++) {
    
    result<<"\n   "<<i<<":";
    
    unsigned int index = traj.i_knotPoints.at(i);
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(index);
    
    result<<"\n       "<<toString(p);
  }


  result<<"\n Points:";
  for(unsigned int i=0;i<traj.trajectory.points.size();i++) {
    result<<"\n\n   Point "<<i<<":";
    
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(i);
  
    result<<"\n"<<toString(p);
  }

  return result.str();
}

