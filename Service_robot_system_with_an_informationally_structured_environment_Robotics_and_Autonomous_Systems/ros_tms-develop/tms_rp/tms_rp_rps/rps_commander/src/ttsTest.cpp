#include <ros/ros.h>
#include <tms_msg_rc/smartpal_speak.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

bool robotSpeak(tms_msg_rc::smartpal_speak::Request& req, tms_msg_rc::smartpal_speak::Response& res)
{
  char cmd[26];
  char tts[128];
  char say[14];

  strcpy(cmd, "pico2wave --wave say.wav ");
  strcpy(say, "aplay say.wav");

  snprintf(tts, 128, "%s\"%s\"", cmd, req.words.c_str());
  system(tts);
  system(say);

  res.success = 1;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smartpal5_speaker");
  ros::NodeHandle nh;
  //~ ros::ServiceServer service  = nh.advertiseService("sp5_speak", robotSpeak);

  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  //--------------------------------------------------------------------------
  while (ros::ok())
  {
    //----------------------------------------------------------------------
    ros::spinOnce();
    loop_rate.sleep();
    //----------------------------------------------------------------------
  }
  return 0;
}
