#ifndef MODIFICATION_REQUEST_HANDLER_H
#define MODIFICATION_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"

class ModificationRequestHandler {
  public:
    ModificationRequestHandler(const ros::NodeHandle& h);
   
    //Cannot make mr const because it has no serialize/deserialize 
    const bool request(ramp_msgs::ModificationRequest& mr);   

  private:
    ros::NodeHandle handle_;
    ros::ServiceClient client_;
   
};

#endif
