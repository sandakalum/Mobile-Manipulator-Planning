#include "utility.h"

Utility::Utility() {}

const uint8_t Utility::getQuadrant(const double angle) const {
  if(angle >= 0 && angle < PI/2) {
    return 1;
  }
  else if(angle >= PI/2 && angle < PI) {
    return 2;
  }
  else if(angle >= -PI && angle < -PI/2) {
    return 3;
  }
  else if(angle >= -PI/2 && angle < 0) {
    return 4;
  }

  return 0;
}


const uint8_t Utility::getQuadrantOfVector(const std::vector<double> v) const {
  return getQuadrant( findAngleToVector(v) );
}



/** This method returns the Euclidean distance between two position vectors */
const double Utility::positionDistance(const std::vector<double> a, const std::vector<double> b) const {

  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} // End euclideanDistance

const double Utility::positionDistance(const trajectory_msgs::JointTrajectoryPoint point_a, const trajectory_msgs::JointTrajectoryPoint point_b) const {
  std::vector<double> a,b;

  a.push_back(point_a.positions.at(0));
  a.push_back(point_a.positions.at(1));

  b.push_back(point_b.positions.at(0));
  b.push_back(point_b.positions.at(1));

  return positionDistance(a,b);
}

const double Utility::findAngleToVector(const std::vector<double> p) const {
  std::vector<double> zero;
  zero.push_back(0); zero.push_back(0);

  return findAngleFromAToB(zero, p);
}


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



const double Utility::findAngleFromAToB(const double x_prev, const double y_prev, const double x, const double y) const {
  std::vector<double> a, b;

  a.push_back(x_prev);
  a.push_back(y_prev);

  b.push_back(x);
  b.push_back(y);

  return findAngleFromAToB(a, b);
}


/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const double Utility::findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const {
  double result;

  // Find the distances in x,y directions and Euclidean distance
  double d_x = b.at(0) - a.at(0);
  double d_y = b.at(1) - a.at(1);
  double euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );

  
  // If the positions are the same,
  // Set the result to the starting orientation if one is provided
  // Or to 0 if no starting orientation is provided
  if(euc_dist <= 0.00001) {
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


/** This method returns distance between orientations a1 and a2. The distance is in the range [-PI, PI]. */
const double Utility::findDistanceBetweenAngles(const double a1, const double a2) const {
  double result;
  double difference = a2 - a1;
  //ROS_INFO("a2: %f, a1: %f diff: %f", a2, a1, difference);
  
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

  for(unsigned int i=0;i<kps.size();i++) {
    result.points.push_back(kps.at(i));
  }

  return result;
}




const ramp_msgs::KnotPoint Utility::getKnotPoint(const ramp_msgs::MotionState ms) const {
  ramp_msgs::KnotPoint result;

  result.motionState = ms;

  return result;
}

const trajectory_msgs::JointTrajectoryPoint Utility::getTrajectoryPoint(const ramp_msgs::MotionState ms) const {
  trajectory_msgs::JointTrajectoryPoint result;

  for(unsigned int i=0;i<ms.positions.size();i++) {
    result.positions.push_back(ms.positions.at(i));
  }

  for(unsigned int i=0;i<ms.velocities.size();i++) {
    result.velocities.push_back(ms.velocities.at(i));
  }

  for(unsigned int i=0;i<ms.accelerations.size();i++) {
    result.accelerations.push_back(ms.accelerations.at(i));
  }

  result.time_from_start = ros::Duration(ms.time);

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
  if(p.accelerations.size() > 0) {
    result<<"\n       Accelerations: ("<<p.accelerations.at(0);
    for(unsigned int k=1;k<p.accelerations.size();k++) {
      result<<", "<<p.accelerations.at(k);
    }
    result<<")";
  }
  
  result<<"\n Time From Start: "<<p.time_from_start;

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::RampTrajectory traj) const 
{
  std::ostringstream result;

  result<<"\n Knot Points:";
  for(unsigned int i=0;i<traj.i_knotPoints.size();i++) 
  {
    result<<"\n   "<<i<<":";
    
    unsigned int index = traj.i_knotPoints.at(i);
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(index);
    
    result<<"\n       "<<toString(p);
  }

  result<<"\n Points:";
  for(unsigned int i=0;i<traj.trajectory.points.size();i++) 
  {
    result<<"\n\n   Point "<<i<<":";
    
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(i);
  
    result<<"\n"<<toString(p);
  }

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
  result<<"\nu_0: "<<bi.u_0<<" u_dot_0: "<<bi.u_dot_0<<" u_dot_max: "<<bi.u_dot_max<<" u_target: "<<bi.u_target;

  return result.str();
}



const std::string Utility::toString(const ramp_msgs::TrajectoryRequest tr) const {
  std::ostringstream result;

  result<<"\nTrajectory Request:\n";
  result<<"\n  Path: "<<toString(tr.path);
  result<<"\n  type: ";
  switch(tr.type) {
    case HOLONOMIC:
      result<<"All Straight Segments";
      break;
    case HYBRID:
      result<<"Partial Bezier";
      break;
    case TRANSITION:
      result<<"Transition";
      break;
    case PREDICTION:
      result<<"Prediction";
  }

  result<<"\nPrint: "<<(tr.print ? "True" : "False");

  for(uint8_t i=0;i<tr.bezierCurves.size();i++) {
    result<<"\n Curve "<<(int)i<<"\n"<<toString(tr.bezierCurves.at(i));
  }

  result<<"\nSegments: "<<(int)tr.segments;
  
  result<<"\n";

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::TrajectoryResponse tr) const
{
  std::ostringstream result;

  result<<"\nTrajectory: "<<toString(tr.trajectory);
  result<<"\nError: "<<(tr.error ? "True" : "False");

  return result.str();
}

const std::string Utility::toString(const ramp_msgs::TrajectorySrv srv) const
{
  std::ostringstream result;
  for(uint8_t i=0;i<srv.request.reqs.size();i++)
  {
    result<<"\nRequest: "<<toString(srv.request.reqs.at(i));
    result<<"\nResponse: "<<toString(srv.response.resps.at(i));
  }
  return result.str();
}
