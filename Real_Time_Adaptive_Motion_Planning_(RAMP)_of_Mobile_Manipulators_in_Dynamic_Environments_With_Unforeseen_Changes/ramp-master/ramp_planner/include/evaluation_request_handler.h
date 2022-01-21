#ifndef EVALUATION_REQUEST_HANDLER_H
#define EVALUATION_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/EvaluationSrv.h"

class EvaluationRequestHandler {
  public:
    EvaluationRequestHandler(const ros::NodeHandle& h);

    //Cannot make mr const because it has no serialize/deserialize 
    const bool request(ramp_msgs::EvaluationSrv& er);
  
  private:
    ros::NodeHandle handle_;
    ros::ServiceClient client_;
};

#endif
