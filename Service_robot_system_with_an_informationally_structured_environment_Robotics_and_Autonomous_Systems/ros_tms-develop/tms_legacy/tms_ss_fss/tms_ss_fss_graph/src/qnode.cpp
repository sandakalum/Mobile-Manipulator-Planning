//------------------------------------------------------------------------------
// @file   : qnode.cpp
// @brief  : QT + ROS node Implementation
// @author : Yoonseok Pyo
// @version: Ver0.6 (since 2012.06.05)
// @date   : 2012.11.26
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <ros/network.h>

#include <string>
#include <sstream>

#include "../include/tms_ss_fss_graph/qnode.hpp"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace fss_graph
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
  ros::init(init_argc, init_argv, "fss_graph");

  if (!ros::master::check())
  {
    return false;
  }
  ros::start();

  ros::NodeHandle nh;
  fss_graph_subscriber = nh.subscribe("fss_tf_data", 10, &QNode::chatterCallback, this);

  start();
  return true;
}

//------------------------------------------------------------------------------
void QNode::chatterCallback(const tms_msg_ss::fss_tf_data::ConstPtr& msg)
{
  tms_msg_ss::fss_tf_data msgTfData;

  int iLrf_scan_max_count = 721;

  for (int i = 0; i < iLrf_scan_max_count; i++)
  {
    m_stMaxErrorData[i].fDistance = msg->fDistance[i];
    m_stMinErrorData[i].fDistance = msg->fDistance[i];
    m_stMaxErrorData[i].fIntensity = msg->fIntensity[i];
    m_stMinErrorData[i].fIntensity = msg->fIntensity[i];
    m_stMaxErrorData[i].fIntrinsicIntensity = msg->fIntrinsicIntensity[i];
    m_stMinErrorData[i].fIntrinsicIntensity = msg->fIntrinsicIntensity[i];
  }

  msgTfData = *msg;
  m_vstTfData.push_back(msgTfData);

  if (m_vstTfData.size() > 400)
    m_vstTfData.erase(m_vstTfData.begin());

  if (m_vstTfData.size() == 400)
  {
    for (uint32_t i = 0; i < m_vstTfData.size(); i++)
    {
      for (uint32_t j = 0; j < m_vstTfData[i].fDistance.size(); j++)
      {
        if (m_vstTfData[i].fDistance[j] > m_stMaxErrorData[j].fDistance)
          m_stMaxErrorData[j].fDistance = m_vstTfData[i].fDistance[j];

        if (m_vstTfData[i].fDistance[j] < m_stMinErrorData[j].fDistance)
          m_stMinErrorData[j].fDistance = m_vstTfData[i].fDistance[j];

        if (m_vstTfData[i].fIntensity[j] > m_stMaxErrorData[j].fIntensity)
          m_stMaxErrorData[j].fIntensity = m_vstTfData[i].fIntensity[j];

        if (m_vstTfData[i].fIntensity[j] < m_stMinErrorData[j].fIntensity)
          m_stMinErrorData[j].fIntensity = m_vstTfData[i].fIntensity[j];

        if (m_vstTfData[i].fIntrinsicIntensity[j] > m_stMaxErrorData[j].fIntrinsicIntensity)
          m_stMaxErrorData[j].fIntrinsicIntensity = m_vstTfData[i].fIntrinsicIntensity[j];

        if (m_vstTfData[i].fIntrinsicIntensity[j] < m_stMinErrorData[j].fIntrinsicIntensity)
          m_stMinErrorData[j].fIntrinsicIntensity = m_vstTfData[i].fIntrinsicIntensity[j];
      }
    }
  }

  for (int i = 0; i < iLrf_scan_max_count; i++)
  {
    m_stTmepData[i].bIsForwardPoint = msg->bIsForwardPoint[i];
    m_stTmepData[i].fDistance = msg->fDistance[i];
    m_stTmepData[i].fIntensity = msg->fIntensity[i];
    m_stTmepData[i].fIntrinsicIntensity = msg->fIntrinsicIntensity[i];
  }
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
}  // namespace tms_msg_ss::
