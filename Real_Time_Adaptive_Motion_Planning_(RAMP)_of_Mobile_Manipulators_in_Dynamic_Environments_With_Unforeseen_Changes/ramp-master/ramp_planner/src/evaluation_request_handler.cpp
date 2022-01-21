#include "evaluation_request_handler.h"


EvaluationRequestHandler::EvaluationRequestHandler(const ros::NodeHandle& h) : handle_(h) 
{
  client_ = handle_.serviceClient<ramp_msgs::EvaluationSrv>("/trajectory_evaluation");
}


const bool EvaluationRequestHandler::request(ramp_msgs::EvaluationSrv& er) 
{
  return client_.call(er);
}
