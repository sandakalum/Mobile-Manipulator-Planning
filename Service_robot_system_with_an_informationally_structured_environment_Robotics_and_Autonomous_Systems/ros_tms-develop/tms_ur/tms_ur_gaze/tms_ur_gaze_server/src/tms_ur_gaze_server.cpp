/*
 * tms_ur_gaze_server.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: kazuto
 */

#include <ros/ros.h>
#include <nodelet/loader.h>

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tms_ur_gaze_server");

  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load("evaluator", "tms_ur_gaze_server::EvaluatorNodelet", remappings, my_argv);
  manager.load("voice_receiver", "tms_ur_gaze_server::VoiceReceiverNodelet", remappings, my_argv);
  ros::waitForShutdown();

  return 0;
}
