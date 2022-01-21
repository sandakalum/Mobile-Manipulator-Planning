#include "prediction.h"

Prediction::Prediction() {}

Prediction::~Prediction() {}




void Prediction::init(const ramp_msgs::TrajectoryRequest req) 
{

  path_ = req.path; 

}


bool Prediction::trajectoryRequest(ramp_msgs::TrajectoryRequest& req, ramp_msgs::TrajectoryResponse& res) 
{
  //ROS_INFO("In Prediction::trajectoryRequest");
  res.trajectory.i_knotPoints.push_back(0);


  std::vector<ramp_msgs::MotionState> traj;
  
  ramp_msgs::MotionState ms_init = req.path.points.at(0).motionState;
  tf::Vector3 v(ms_init.velocities.at(0), ms_init.velocities.at(1), 0);
  double vNorm = sqrt(v.dot(v));
  double w = ms_init.velocities.at(2);

  if(fabs(w) < 0.01 && fabs(vNorm) < 0.01) 
  {
    //ROS_INFO("No velocity, Pushing on one point");
    traj.push_back(ms_init);
  }
  
  else if(fabs(w) > 0.01 && fabs(vNorm) < 0.01) 
  {
    //ROS_INFO("Self-rotating, Pushing on one point");
    traj.push_back(ms_init);
  }

  else if(fabs(req.path.points.at(0).motionState.velocities.at(2)) < 0.01) 
  {
    //ROS_INFO("In straight line prediction");
    Line li;


    li.init(req.path.points.at(0).motionState, req.path.points.at(1).motionState);
    traj = li.generatePoints(); 
  }

  else if(fabs(req.path.points.at(0).motionState.velocities.at(2)) > 0.01 ) 
  {
    //ROS_INFO("In circle prediction");
    Circle ci;
    ci.init(req.path.points.at(0).motionState);
    traj = ci.generatePoints(); 
  }
  else 
  {
    //ROS_INFO("In else");
    traj.push_back(req.path.points.at(0).motionState);
  }

  //ROS_INFO("Done with building path");

  ramp_msgs::RampTrajectory rt;
  for(int i=0;i<traj.size();i++) 
  {
    rt.trajectory.points.push_back(utility_.getTrajectoryPoint(traj.at(i)));
  }
  res.trajectory = rt;
  res.trajectory.i_knotPoints.push_back(0);
  res.trajectory.i_knotPoints.push_back(rt.trajectory.points.size()-1);

  //ROS_INFO("Predicted trajectory: %s", utility_.toString(rt).c_str());


  return true;
}


