#include "utility.h"

Utility::Utility() {
}


/** This method returns the Euclidean distance between two position vectors */
const double Utility::positionDistance(const std::vector<double> a, const std::vector<double> b) const 
{
  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} // End euclideanDistance




const double Utility::findAngleFromAToB(const trajectory_msgs::JointTrajectoryPoint a, const trajectory_msgs::JointTrajectoryPoint b) const {
  std::vector<double> c;
  std::vector<double> d;
  
  // Set c
  for(unsigned int i=0;i<a.positions.size();i++) {
    c.push_back(a.positions.at(i));
  }
  
  //Set d
  for(unsigned int i=0;i<b.positions.size();i++) {
    d.push_back(b.positions.at(i));
  }

  return findAngleFromAToB(c, d);
}

const double Utility::findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const {
  std::vector<double> d_a;
  std::vector<double> d_b;
  
  for(unsigned int i=0;i<a.size();i++) {
    d_a.push_back(a.at(i));
  }
  
  for(unsigned int i=0;i<b.size();i++) {
    d_b.push_back(b.at(i));
  }

  return findAngleFromAToB(d_a, d_b);
}

/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const double Utility::findAngleFromAToB(const std::vector<double>& a, const std::vector<double>& b) const {
  double result;

  // Find the distances in x,y directions and Euclidean distance
  double d_x = b[0] - a[0];
  double d_y = b[1] - a[1];
  double euc_dist = sqrt( (d_x*d_x) + (d_y*d_y) );
  
  // If the positions are the same,
  // Set the result to the starting orientation if one is provided
  // Or to 0 if no starting orientation is provided
  if(euc_dist <= 0.0001) {
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

  return result;
} // End findAngleFromAToB


const double Utility::findAngleFromAToB(const double x_prev, const double y_prev, const double x, const double y) const {
  std::vector<double> a, b;

  a.push_back(x_prev);
  a.push_back(y_prev);

  b.push_back(x);
  b.push_back(y);

  return findAngleFromAToB(a, b);
}


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
} // End findDistanceBetweenAngles



const double Utility::displaceAngle(const double a1, double a2) const {

  a2 = fmodf(a2, 2*PI);

  if(a2 > PI) {
    a2 = fmodf(a2,PI) - PI;
  }

  return findDistanceBetweenAngles(-a1, a2);
} // End displaceAngle



/** a and b must be the same size */
const double Utility::getEuclideanDist(const std::vector<double> a, const std::vector<double> b) const {
  double result=0;

  for(unsigned int i=0;i<a.size();i++) {
    result += pow(a.at(i) - b.at(i), 2);
  }
  
  result = sqrt(result);
  return result;
}



const ramp_msgs::Path Utility::getPath(const std::vector<ramp_msgs::MotionState> mps) const {
  ramp_msgs::Path result;

  for(unsigned int i=0;i<mps.size();i++) {
    ramp_msgs::KnotPoint kp;
    kp.motionState = mps.at(i);
    kp.stopTime = 0;
    result.points.push_back(kp);
  }

  return result;
}



const ramp_msgs::Path Utility::getPath(const std::vector<ramp_msgs::KnotPoint> kps) const {
  ramp_msgs::Path result;

  for(unsigned int i=0;i<kps.size();i++) 
  {
    result.points.push_back(kps.at(i));
  }

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
  if(p.positions.size() > 0)
  {
    result<<"\n       Positions: ("<<p.positions.at(0);
    for(unsigned int k=1;k<p.positions.size();k++) {
      result<<", "<<p.positions.at(k);
    }
    result<<")";
  }

  //Velocities
  if(p.velocities.size() > 0) 
  {
    result<<"\n       Velocities: ("<<p.velocities.at(0);
    for(unsigned int k=1;k<p.velocities.size();k++) {
      result<<", "<<p.velocities.at(k);
    }
    result<<")";
  }
  
  //Accelerations
  if(p.accelerations.size() > 0) 
  {
    result<<"\n       Accelerations: ("<<p.accelerations.at(0);
    for(unsigned int k=1;k<p.accelerations.size();k++) {
      result<<", "<<p.accelerations.at(k);
    }
    result<<")";
  }
  
  result<<"\n Time From Start: "<<p.time_from_start;

  return result.str();
}
    

const std::string Utility::toString(const ramp_msgs::BezierCurve bi) const {
  std::ostringstream result;

  result<<"\nSegment Points: ";
  for(uint8_t i=0;i<bi.segmentPoints.size();i++) {
    result<<"\n"<<toString(bi.segmentPoints.at(i));
  }

  result<<"\nControl Points: ";
  for(uint8_t i=0;i<bi.controlPoints.size();i++) {
    result<<"\n"<<toString(bi.controlPoints.at(i));
  }

  result<<"\nms_maxVA: "<<toString(bi.ms_maxVA);
  result<<"\nms_initialVA: "<<toString(bi.ms_initialVA);
  result<<"\nms_begin: "<<toString(bi.ms_begin);
  result<<"\nl: "<<bi.l;
  result<<"\nu_0: "<<bi.u_0<<" u_dot_0: "<<bi.u_dot_0;
  result<<"\nu_target: "<<bi.u_target;

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::RampTrajectory traj) const {
  std::ostringstream result;

  result<<"\n Knot Points:";

  for(unsigned int i=0;i<traj.i_knotPoints.size();i++) {
    result<<"\n   "<<i<<":";
    
    unsigned int index = traj.i_knotPoints.at(i);
    if(index > traj.trajectory.points.size()-1) 
    {
      ROS_ERROR("index: %i, traj.points.size(): %i", (int)index, (int)traj.trajectory.points.size());
    }
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(index);
    
    result<<"\n       "<<toString(p);
  }

  result<<"\nNumber of curves: "<<traj.curves.size();
  if(traj.curves.size() > 0)
  {
    result<<"\nCurve: "<<toString(traj.curves[0]);
  }

  result<<"\nt_start: "<<traj.t_start;

  /*result<<"\n Points:";
  //for(unsigned int i=15;i<27;i++) {
  //for(unsigned int i=0;i<7;i++) {
  //for(unsigned int i=0;i<traj.trajectory.points.size();i++) {
  //for(unsigned int i=0;i<traj.trajectory.points.size();i++) {
    //ROS_INFO("i: %i", (int)i);
    result<<"\n\n   Point "<<i<<":";
    
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(i);
  
    result<<"\n"<<toString(p);
  }*/
  //ROS_INFO("Done with points");


  /*for(uint8_t i=0;i<traj.curves.size();i++) {
    //ROS_INFO("curve %i", (int)i);
    result<<"\n Curve "<<(int)i<<"\n"<<toString(traj.curves.at(i));
  }*/

  //ROS_INFO("Done with curves");

  return result.str();
}
