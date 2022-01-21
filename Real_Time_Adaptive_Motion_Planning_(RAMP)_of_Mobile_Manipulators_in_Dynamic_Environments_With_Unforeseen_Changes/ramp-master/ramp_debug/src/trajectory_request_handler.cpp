#include "trajectory_request_handler.h"


TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : handle_(h) 
{
  client_ = handle_.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");
}


const bool TrajectoryRequestHandler::request(ramp_msgs::TrajectorySrv& tr) 
{
  return client_.call(tr);
}
