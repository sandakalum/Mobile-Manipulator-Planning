//------------------------------------------------------------------------------
// @file   : qnode.cpp
// @brief  : QT + ROS node Implementation
// @author : Yoonseok Pyo, Masahide Tanaka
// @version: Ver0.4 (since 2012.06.05)
// @date   : 2012.11.26
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
// Namespaces
//------------------------------------------------------------------------------
namespace bas_graph
{
//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
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
  ros::init(init_argc, init_argv, "bas_graph");

  if (!ros::master::check())
  {
    return false;
  }
  ros::start();

  ros::NodeHandle nh;
  bas_graph_subscriber = nh.subscribe("bas_behavior_data", 10, &QNode::behaviorCallback, this);

  start();
  return true;
}

//------------------------------------------------------------------------------
void QNode::behaviorCallback(const tms_msg_ss::bas_behavior_data::ConstPtr& msg)
{
  m_bas_behavior_data = *msg;
}

//------------------------------------------------------------------------------
void QNode::run()
{
  while (ros::ok())
  {
    ros::spin();
  }
  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

//------------------------------------------------------------------------------
}  // namespace bas_graph

//------------------------------------------------------------------------------
