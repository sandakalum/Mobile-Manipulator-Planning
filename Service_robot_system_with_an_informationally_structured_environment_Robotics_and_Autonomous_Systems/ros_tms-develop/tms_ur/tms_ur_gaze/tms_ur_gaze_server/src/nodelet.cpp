/*
 * nodelet.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: kazuto
 */

#include "evaluator.h"
#include "voice_receiver.h"

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace tms_ur_gaze_server
{
//----------------------------------------------------------------------------------
// EvaluatorNodelet
//----------------------------------------------------------------------------------
class EvaluatorNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  boost::shared_ptr< Evaluator > evaluator_;

public:
  virtual void onInit()
  {
    NODELET_INFO("evaluator nodelet");
    ros::AsyncSpinner spinner(0);
    nh_ = getNodeHandle();
    evaluator_.reset(new Evaluator(&nh_));
    spinner.start();
  }
};
PLUGINLIB_EXPORT_CLASS(tms_ur_gaze_server::EvaluatorNodelet, nodelet::Nodelet)

//----------------------------------------------------------------------------------
// VoiceReceiverNodelet
//----------------------------------------------------------------------------------
class VoiceReceiverNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  boost::shared_ptr< VoiceReceiver > voice_receiver_;

public:
  virtual void onInit()
  {
    NODELET_INFO("voice receiver nodelet");
    ros::AsyncSpinner spinner(0);
    nh_ = getNodeHandle();
    voice_receiver_.reset(new VoiceReceiver(&nh_));
    spinner.start();
  }
};
PLUGINLIB_EXPORT_CLASS(tms_ur_gaze_server::VoiceReceiverNodelet, nodelet::Nodelet)
}
