//------------------------------------------------------------------------------
// @file   : qnode.cpp
// @brief  : qt node function
// @author : Yoonseok Pyo, Masahide Tanaka
// @version: Ver0.9.5 (since 2012.05.17)
// @date   : 2012.11.14
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <ros/network.h>

#include <string>
#include <sstream>

#include "qnode.hpp"

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
QNode::QNode(int argc, char **argv, const std::string &name) : init_argc(argc), init_argv(argv), node_name(name)
{
}

//------------------------------------------------------------------------------
QNode::~QNode()
{
  shutdown();
}

//------------------------------------------------------------------------------
void QNode::shutdown()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

//------------------------------------------------------------------------------
bool QNode::init()
{
  ros::init(init_argc, init_argv, node_name);

  if (!ros::master::check())
  {
    return false;
  }
  ros::start();
  start();
  return true;
}

//------------------------------------------------------------------------------
void QNode::run()
{
  // ros::Rate loop_rate(40);     // 0.025sec
  while (ros::ok())
  {
    ros::spin();
    // ros::spinOnce();
    // loop_rate.sleep();
  }
  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

//------------------------------------------------------------------------------
