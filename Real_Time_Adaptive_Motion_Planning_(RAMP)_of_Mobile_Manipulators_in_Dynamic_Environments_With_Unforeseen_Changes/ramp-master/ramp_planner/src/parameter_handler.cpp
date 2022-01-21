#include "parameter_handler.h"

ParameterHandler::ParameterHandler() {}


void ParameterHandler::setImminentCollision(bool ic) 
{ 
  ros::param::set("imminent_collision", ic);
}

void ParameterHandler::setCCStarted(bool cc) 
{ 
  ros::param::set("ramp/cc_started", cc);
}

void ParameterHandler::setTestCase(bool ready_tc)
{
  ros::param::set("ramp/ready_tc", ready_tc);
}
