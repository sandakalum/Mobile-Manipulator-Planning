#ifndef PARAMETER_HANDLER_H
#define PARAMETER_HANDLER_H
#include "ros/ros.h"


class ParameterHandler {
  public:
    ParameterHandler(); 

    void setImminentCollision(bool ic); 
    
    void setCCStarted(bool cc); 
  
    void setTestCase(bool ready_tc);
  private:
};

#endif
