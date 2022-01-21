//------------------------------------------------------------------------------
// @file   : simpletask_service.cpp
// @brief  : service for simple tasks
// @author : Alaoui Hassani Atlas Omar
// @version: Ver1.0.0 (since 2014.06.16)
// @date   : 2014.06.17
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <unistd.h>
#include <tms_msg_rs/robot_task.h>
#include <tms_msg_rs/rs_task.h>

#include <sstream>
#include <string>

//------------------------------------------------------------------------------
using namespace std;

//------------------------------------------------------------------------------
// Global variables for data transfer
int id;
string task = "";

//------------------------------------------------------------------------------
// Service funciton : send data
bool robot_task(tms_msg_rs::rs_task::Request& req, tms_msg_rs::rs_task::Response& res)
{
  if (req.id = id)
  {
    res.data.id = id;
    res.data.task = task;
    res.result = 1;
    return true;
  }
  else
  {
    res.result = -1;
    return false;
  }
}

// Get the data from the message (user request)
void sendTask(const tms_msg_rs::robot_task& msg)
{
  id = msg.id;
  task = msg.task;
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simpletask_service");
  ros::NodeHandle nh;
  ros::Subscriber task_sub = nh.subscribe("rs_simple_task", 10, sendTask);
  ros::ServiceServer send_task = nh.advertiseService("rs_task", robot_task);
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
// EOF