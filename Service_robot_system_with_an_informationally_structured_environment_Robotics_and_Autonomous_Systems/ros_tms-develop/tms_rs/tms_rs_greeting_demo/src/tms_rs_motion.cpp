/*
 * tms_rs_motion.cpp
 *
 *  Created on: 2014/08/01
 *      Author: hashiguchi
 */
#include <ros/ros.h>
#include <string>
#include <iostream>

#include <tms_msg_rc/smartpal_control.h>

#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_LUMBA 6

#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

ros::ServiceClient motion_client;

bool sp_control_function(int unit, int cmd, int arg_size, double* arg)
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

bool sp5_control(int num, const double* arg)
{
  // num 0:vehicle&Rarm, 1:Rarm, 2:Rarm&waist, 3:waist, 4:LRarm, 5:LRarm
  if (num == 0)
  {
    double arg_vehicle[3] = {0, 0, arg[0]};
    sp_control_function(UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle);
  }
  if (num == 0 || num == 1 || num == 2 || num == 4 || num == 5)
  {
    double arg_armR[8] = {arg[3], arg[4], arg[5], arg[6], arg[7], arg[8], arg[9], 30.0};
    sp_control_function(UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR);
  }

  if (num == 4 || num == 5)
  {
    double arg_armL[8] = {arg[11], arg[12], arg[13], arg[14], arg[15], arg[16], arg[17], 30.0};
    sp_control_function(UNIT_ARM_L, CMD_MOVE_ABS, 8, arg_armL);
  }

  if (num == 2 || num == 3)
  {
    double arg_waist[4] = {arg[1], arg[2], 20.0, 10.0};
    sp_control_function(UNIT_LUMBA, CMD_MOVE_REL, 4, arg_waist);
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tms_rs_motion");
  ros::NodeHandle n;

  motion_client = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");

  const double motion_ini[19] = {0.0, 0.0, 0.0,   0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, -10.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0};
  const double motion0[19] = {0.0, 0.0, 0.0,   1.1, -14.3, 56.6, 120.5, 0.0, 23.5, 0.0,
                              0.0, 0.0, -10.0, 0.0, 0.0,   0.0,  0.0,   0.0, 0.0};
  const double motion5[19] = {0.0, -6.0, 30.0,  4.7, -45.2, -49.0, 120.5, 23.5, 15.5, -22.4,
                              0.0, 0.0,  -10.0, 0.0, 0.0,   0.0,   0.0,   0.0,  0.0};
  const double motion6[19] = {0.0, 0.0, 0.0,   4.7, -45.2, -49.0, 120.5, 23.5, 15.5, -22.4,
                              0.0, 0.0, -10.0, 0.0, 0.0,   0.0,   0.0,   0.0,  0.0};
  const double motion7[19] = {0.0, 0.0, 0.0,   0.0,  -22.2, 41.4, 40.0, 64.3, 5.5, 0.0,
                              0.0, 0.0, -22.2, 41.4, 40.0,  64.3, 5.5,  0.0,  0.0};
  const double motion9[19] = {0.0, 0.0,  0.0,   0.0,  -10.0, 0.0,  0.0, 0.0,   0.0, 0.0,
                              0.0, 55.7, -25.5, 23.5, 115.0, 77.0, 0.0, -30.0, 0.0};
  const double motion12[19] = {-45.0, 0.0, 0.0,   0.0, -24.0, 101.0, 35.8, 65.5, 0.0, -17.7,
                               0.0,   0.0, -10.0, 0.0, 0.0,   0.0,   0.0,  0.0,  0.0};  // Open,Close
  const double motion13[19] = {45.0, 0.0, 0.0,   0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, -10.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0};

  while (ros::ok())
  {
    int motion_num;

    while (1)
    {
      std::cout << "motionNum 0:Init, 1:Hello, 2:ThankYou, 3:PleaseEnjoy,"
                   " 4:DoMyBest, 5:Open(Close)Door, 10000:quit" << std::endl;
      std::cin >> motion_num;

      if (motion_num == 10000)
        goto QUIT;
      if (!std::cin.fail() && (0 <= motion_num && motion_num <= 5))
        break;

      std::cin.clear();
      std::cin.ignore(1024, '\n');
    }

    switch (motion_num)
    {
      case 0:
        // initial
        if (sp5_control(5, motion_ini))
        {
        };
        break;
      case 1:
        // "Hello, I am Smart pal five."
        if (sp5_control(1, motion0))
          ROS_INFO("succeed to call service motion0");
        break;
      case 2:
        // "Thank you for coming to our laboratory today."
        if (sp5_control(2, motion5))
          ROS_INFO("succeed to call service motion2_1");
        if (sp5_control(2, motion6))
          ROS_INFO("succeed to call service motion2_2");  // return waist
        break;
      case 3:
        // "Please Enjoy watching my demonstration."
        if (sp5_control(4, motion7))
          ROS_INFO("succeed to call service motion3");
        break;
      case 4:
        // "I will do my best."
        if (sp5_control(4, motion9))
          ROS_INFO("succeed to call service motion4");
        break;
      case 5:
        // "Open(Close) the door."
        if (sp5_control(0, motion12))
          ROS_INFO("succeed to call service motion5_1");
        ros::Duration(3.0).sleep();
        if (sp5_control(0, motion13))
          ROS_INFO("succeed to call service motion5_2");

        break;
      default:
        break;
    }
  }
QUIT:
  return 0;
}
