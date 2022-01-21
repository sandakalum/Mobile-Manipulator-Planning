#include "modification_request_handler.h"

ModificationRequestHandler::ModificationRequestHandler(const ros::NodeHandle& h) : handle_(h) 
{
  client_ = handle_.serviceClient<ramp_msgs::ModificationRequest>("/path_modification");
}


const bool ModificationRequestHandler::request(ramp_msgs::ModificationRequest& mr) 
{
  return client_.call(mr);
}
