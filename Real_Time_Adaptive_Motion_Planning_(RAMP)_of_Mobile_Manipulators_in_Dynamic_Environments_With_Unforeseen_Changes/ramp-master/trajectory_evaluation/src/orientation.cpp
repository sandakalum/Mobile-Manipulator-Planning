#include "orientation.h"

Orientation::Orientation() : Q_(1000. / PI) {}


const double Orientation::getDeltaTheta(const ramp_msgs::RampTrajectory& trj) const
{
  ros::Time t_start = ros::Time::now();

  double result = 0.f;

  if(trj.i_knotPoints.size() > 1) 
  {

    trajectory_msgs::JointTrajectoryPoint a = trj.trajectory.points.at(0);
    trajectory_msgs::JointTrajectoryPoint b = trj.trajectory.points.at(trj.i_knotPoints.at(1));
    ////ROS_INFO("a: %s\nb: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
    
    double thetaNec = utility_.findAngleFromAToB(a, b);
    
    if(trj.t_start.toSec() < 0.01)
    {
      result          = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );
    }
    else
    {
      result          = fabs( utility_.findDistanceBetweenAngles(theta_at_cc_, thetaNec) );
    }

    ////ROS_INFO("thetaNec: %f result: %f", thetaNec, result);
  }

  ////ROS_INFO("t_getDeltaTheta: %f", (ros::Time::now()-t_start).toSec());
  return result;
}


const double Orientation::perform(const ramp_msgs::RampTrajectory& trj) 
{
  //ROS_INFO("In Orientation::perform");
  ////ROS_INFO("Trj: %s", utility_.toString(trj).c_str());
  ////ROS_INFO("trajectory_.i_knotPoints.size(): %i", (int)trj.i_knotPoints.size());
  double result = 0.;

  // Add the change in orientation needed to move on this trajectory
  // Check if there is more than 1 point
  if(trj.i_knotPoints.size() > 1) 
  {

    trajectory_msgs::JointTrajectoryPoint a = trj.trajectory.points.at(0);
    trajectory_msgs::JointTrajectoryPoint b = trj.trajectory.points.at(trj.i_knotPoints.at(1));
    //ROS_INFO("a: %s\nb: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
    
    double thetaNec = utility_.findAngleFromAToB(a, b);   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );
    //ROS_INFO("thetaNec: %f currentTheta_: %f deltaTheta: %f", thetaNec, currentTheta_, deltaTheta);

    double normalize = PI;
    deltaTheta /= normalize;

    // Normalize
    result += deltaTheta;
  }

  return result;
}


const double Orientation::getPenalty(const ramp_msgs::RampTrajectory& trj) const 
{
  double result = 0.;

  if(trj.i_knotPoints.size() > 1) 
  {
    double thetaNec = utility_.findAngleFromAToB(trj.trajectory.points.at(0),
      trj.trajectory.points.at( trj.i_knotPoints.at(1) ));   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );

    double mag_linear = sqrt( pow(trj.trajectory.points.at(0).velocities.at(0), 2) + 
        pow(trj.trajectory.points.at(0).velocities.at(1), 2) );

    ////ROS_INFO("thetaNec: %f deltaTheta: %f mag_linear: %f", thetaNec, deltaTheta, mag_linear);

    // If delta theta is too high, add a penalty
    ////ROS_INFO("Adding penalty for deltaTheta: %f", deltaTheta);
    double normalize = PI;
    deltaTheta /= normalize;
    result += (Q_ * normalize);
  } // end if > 1 knot point


  ////ROS_INFO("trajectory.size(): %i", (int)trj.trajectory.points.size());
  if(trj.trajectory.points.size() > 2)
  {
    trajectory_msgs::JointTrajectoryPoint p = trj.trajectory.points.at(2);
    double v = sqrt( pow(p.velocities.at(0), 2) + pow(p.velocities.at(1), 2) );
    double w = p.velocities.at(2);

    if(fabs(v) < 0.0001 && fabs(w) > 0.01)
    {
      result += 1000;
    }
  }

  return result;
}
