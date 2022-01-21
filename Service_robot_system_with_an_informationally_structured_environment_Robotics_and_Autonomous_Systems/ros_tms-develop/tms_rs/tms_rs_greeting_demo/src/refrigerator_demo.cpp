/*
 * refrigerator_demo.cpp
 *
 *  Created on: 2014/07/31
 *      Author: hashiguchi
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <tms_msg_rs/rs_home_appliances.h>
#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_rc/robot_tts.h>

#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3

#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

ros::ServiceClient motion_client;
ros::ServiceClient speech_client;
ros::ServiceClient refrigerator_client;

using namespace boost::posix_time;
time_duration const td1 = seconds(1);

// motion
const double Mturn[19] = {-36.0, 0.0, 0.0,   0.0, -24.0, 101.0, 35.8, 65.5, 0.0, -17.7,
                          0.0,   0.0, -10.0, 0.0, 0.0,   0.0,   0.0,  0.0,  0.0};
const double Mreturn[19] = {36.0, 0.0, 0.0,   0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0,  0.0, -10.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0};

const double arg_vehicle[3] = {0.0, 0.0, Mturn[0]};
const double arg_armR[8] = {Mturn[3], Mturn[4], Mturn[5], Mturn[6], Mturn[7], Mturn[8], Mturn[9], 30.0};
const double arg_vehicle2[3] = {0.0, 0.0, Mreturn[0]};
const double arg_armR2[8] = {Mreturn[3], Mreturn[4], Mreturn[5], Mreturn[6], Mreturn[7], Mreturn[8], Mreturn[9], 30.0};

// speech
const std::string open_ref("Open the door.");
const std::string close_ref("Close the door.");

bool sp5_control_caller(int unit, int cmd, int arg_size, const double *arg)
{
  tms_msg_rc::smartpal_control sp_control_srv;

  sp_control_srv.request.unit = unit;
  sp_control_srv.request.cmd = cmd;
  sp_control_srv.request.arg.resize(arg_size);
  for (int i = 0; i < sp_control_srv.request.arg.size(); i++)
  {
    sp_control_srv.request.arg[i] = arg[i];
  }

  if (motion_client.call(sp_control_srv))
    ROS_INFO("result: %d", sp_control_srv.response.result);
  else
    ROS_ERROR("Failed to call service sp5_control");

  return true;
}

bool sp5_tts_caller(std::string sentene)
{
  tms_msg_rc::robot_tts sp_tts_srv;

  sp_tts_srv.request.text = sentene;
  if (speech_client.call(sp_tts_srv))
  {
    ROS_INFO("result: %d", sp_tts_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service smartpal5_tts");
    return false;
  }
  return true;
}

bool refrigerator_control_caller(int service)
{
  tms_msg_rs::rs_home_appliances ref_control_srv;

  ref_control_srv.request.id = 2009;
  ref_control_srv.request.service = service;
  if (refrigerator_client.call(ref_control_srv))
    ROS_INFO("result: %d", ref_control_srv.response.result);
  else
  {
    ROS_ERROR("Failed to call service refrigerator_control");
    return false;
  }
  return true;
}

bool callback(tms_msg_rs::rs_home_appliances::Request &req, tms_msg_rs::rs_home_appliances::Response &res)
{
  if (req.service == 0)
  {  // Close
    boost::thread th_motion1(boost::bind(&sp5_control_caller, UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle));
    boost::thread th_speech(boost::bind(&sp5_tts_caller, close_ref));
    bool const has_completed = th_motion1.timed_join(td1);
    boost::thread th_motion2(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR));
    boost::thread th_refrigerator(boost::bind(&refrigerator_control_caller, 0));

    ros::Duration(3.0).sleep();
    boost::thread th_remotion1(boost::bind(&sp5_control_caller, UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle2));
    bool const has_completed1 = th_remotion1.timed_join(td1);
    boost::thread th_remotion2(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR2));
  }
  else if (req.service == 1)
  {  // Open
    boost::thread th_motion1(boost::bind(&sp5_control_caller, UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle));
    boost::thread th_speech(boost::bind(&sp5_tts_caller, open_ref));
    bool const has_completed = th_motion1.timed_join(td1);
    boost::thread th_motion2(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR));
    boost::thread th_refrigerator(boost::bind(&refrigerator_control_caller, 1));

    ros::Duration(3.0).sleep();
    boost::thread th_remotion1(boost::bind(&sp5_control_caller, UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle2));
    bool const has_completed1 = th_remotion1.timed_join(td1);
    boost::thread th_remotion2(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR2));
  }
  else
  {
    ROS_ERROR("Illegal service number");
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ref_demo");
  ros::NodeHandle n;

  motion_client = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  speech_client = n.serviceClient< tms_msg_rc::robot_tts >("smartpal5_tts");
  refrigerator_client = n.serviceClient< tms_msg_rs::rs_home_appliances >("refrigerator_controller");

  ros::ServiceServer service = n.advertiseService("ref_demo", callback);

  ROS_INFO("Ready to demonstrate refrigerator.");
  ros::spin();

  return 0;
}
