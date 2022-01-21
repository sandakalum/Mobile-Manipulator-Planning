/*
 * speech.cpp
 *
 *  Created on: 2014/07/30
 *      Author: hashiguchi
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>

#include <tms_msg_rc/robot_tts.h>

ros::ServiceClient speech_client;

// bool sp5_tts(std::string* sentene, double* timing, int size) {
//	tms_msg_rc::robot_tts sp_tts_srv;
//
//	for (int i=0; i<size; i++) {
//		sp_tts_srv.request.text = sentene[i];
//		if (speech_client.call(sp_tts_srv)) {ROS_INFO("result: %d", sp_tts_srv.response.result);}
//		else {
//			ROS_ERROR("Failed to call service smartpal5_tts");
//			return false;
//		}
//		ros::Duration(timing[i]).sleep();
//	}
//	return true;
//}

bool sp5_tts(std::string sentene)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tms_rs_speech");
  ros::NodeHandle n;
  speech_client = n.serviceClient< tms_msg_rc::robot_tts >("smartpal5_tts");

  //	int size = 4;
  //	std::string s_sentence_list[4] = {"Hello, I am Smart pal five.", "Thank you for coming to our rabo.",
  //			"Please enjoy watching my demo.", "I will do my best."};
  //	double timing_list[4] = {9.0, 9.0, 7.0, 3.0};
  //
  //	sp5_tts(s_sentence_list, timing_list, size);

  const std::string hello("Hello, I am Smart pal five.");
  const std::string thank_you("Thank you for coming to our rabo.");
  const std::string enjoy("Please enjoy watching my demo.");
  const std::string do_my_best("I will do my best.");
  const std::string open("Open the door.");
  const std::string close("Close the door.");

  while (ros::ok())
  {
    int speech_num;
    while (1)
    {
      std::cout << "speechNum 1:Hello, 2:ThankYou, 3:PleaseEnjoy, "
                   "4:DoMyBest, 5:OpenDoor, 6:CloseDoor, 10000:quit" << std::endl;
      std::cin >> speech_num;

      if (speech_num == 10000)
        goto QUIT;
      if (!std::cin.fail() && (1 <= speech_num && speech_num <= 6))
        break;

      std::cin.clear();
      std::cin.ignore(1024, '\n');
    }

    switch (speech_num)
    {
      case 1:
        if (sp5_tts(hello))
          ROS_INFO("succeed to call service speech1");
        break;
      case 2:
        if (sp5_tts(thank_you))
          ROS_INFO("succeed to call service speech2");
        break;
      case 3:
        if (sp5_tts(enjoy))
          ROS_INFO("succeed to call service speech3");
        break;
      case 4:
        if (sp5_tts(do_my_best))
          ROS_INFO("succeed to call service speech4");
        break;
      case 5:
        if (sp5_tts(open))
          ROS_INFO("succeed to call service speech5");
        break;
      case 6:
        if (sp5_tts(close))
          ROS_INFO("succeed to call service speech6");
        break;
      default:
        break;
    }
  }
QUIT:
  return 0;
}
