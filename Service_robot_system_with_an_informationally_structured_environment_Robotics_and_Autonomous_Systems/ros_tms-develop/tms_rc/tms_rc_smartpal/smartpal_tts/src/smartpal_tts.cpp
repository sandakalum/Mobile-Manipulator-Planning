///-----------------------------------------------------------------------------
/// @FileName smartpal5_control.cpp
/// @Date 2014.01.02 / 2013.01.27
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <ros/ros.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <tms_msg_rc/robot_tts.h>

//------------------------------------------------------------------------------
bool robotTts(tms_msg_rc::robot_tts::Request &req, tms_msg_rc::robot_tts::Response &res)
{
  char cmd[26];
  char tts[128];
  char say[14];

  strcpy(cmd, "pico2wave --wave ~/say.wav ");
  strcpy(say, "aplay ~/say.wav");

  snprintf(tts, 128, "%s\"%s\"", cmd, req.text.c_str());
  system(tts);
  system(say);

  res.result = 1;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  ros::init(argc, argv, "smartpal5_tts");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("smartpal5_tts", robotTts);

  //--------------------------------------------------------------------------
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)
  while (ros::ok())
  {
    //----------------------------------------------------------------------
    ros::spinOnce();
    loop_rate.sleep();
    //----------------------------------------------------------------------
  }
  return 0;
}

//------------------------------------------------------------------------------
