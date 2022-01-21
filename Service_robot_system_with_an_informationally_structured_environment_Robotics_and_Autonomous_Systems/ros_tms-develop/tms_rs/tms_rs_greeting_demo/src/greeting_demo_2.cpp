#include <ros/ros.h>
#include <string>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <tms_msg_rc/smartpal_control.h>

#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_LUMBA 6

#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

ros::ServiceClient motion_client2;

// timeout
using namespace boost::posix_time;
time_duration const td1 = seconds(1);
time_duration const td4 = seconds(4);
time_duration const td5 = seconds(5);
time_duration const td6 = seconds(6);
time_duration const td7 = seconds(7);
time_duration const td15 = seconds(15);

// motion
const double aArmIni[8] = {0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0};
const double aWaistIni[3] = {0.0, 15.0, 10.0};
const double aWaistIni4[4] = {0.0, 0.0, 15.0, 10.0};
const double aArmHello[8] = {1.1, -14.3, 56.6, 120.5, 0.0, 23.5, 0.0, 30.0};
const double aArmThankYou[8] = {4.7, -45.2, -49.0, 120.5, 23.5, 15.5, -22.4, 30.0};
const double aWaistThankYou[4] = {-6.0, 30.0, 20.0, 10.0};
const double aRArmEnjoy[8] = {0.0, -22.2, 41.4, 40.0, 64.3, 5.5, 0.0, 30.0};
const double aLArmEnjoy[8] = {0.0, -22.2, 41.4, 40.0, 64.3, 5.5, 0.0, 30.0};
const double aArmDoMyBest[8] = {55.7, -25.5, 23.5, 115.0, 77.0, 0.0, -30.0, 30.0};

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

  if (motion_client2.call(sp_control_srv))
    ROS_INFO("result: %d", sp_control_srv.response.result);
  else
    ROS_ERROR("Failed to call service sp5_control");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "greeting_demo2");
  ros::NodeHandle n;

  motion_client2 = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_2control");

  while (ros::ok())
  {
    int num;
    while (1)
    {
      std::cout << "Num 0:Initial/1:Hello/2:ThankYou/3:PleaseEnjoy/"
                   "4:DoMyBest/5:All/10000:quit" << std::endl;
      std::cout << "Num : ";
      std::cin >> num;

      if (num == 10000)
        goto QUIT;
      if (!std::cin.fail() && (0 <= num && num <= 5))
        break;

      std::cin.clear();
      std::cin.ignore(1024, '\n');
    }

    switch (num)
    {
      case 0:
      {
        boost::thread th_motion01(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmIni));
        boost::thread th_motion02(boost::bind(&sp5_control_caller, UNIT_ARM_L, CMD_MOVE_ABS, 8, aArmIni));
        break;
      }
      case 1:
      {
        boost::thread th_motion11(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmHello));
        break;
      }
      case 2:
      {
        boost::thread th_motion21(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmThankYou));
        boost::thread th_motion22(boost::bind(&sp5_control_caller, UNIT_LUMBA, CMD_MOVE_REL, 4, aWaistThankYou));
        th_motion22.join();
        boost::thread th_motion23(boost::bind(&sp5_control_caller, UNIT_LUMBA, CMD_MOVE_ABS, 3, aWaistIni));
        break;
      }
      case 3:
      {
        boost::thread th_motion31(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aRArmEnjoy));
        break;
      }
      case 4:
      {
        boost::thread th_motion42(boost::bind(&sp5_control_caller, UNIT_ARM_L, CMD_MOVE_ABS, 8, aArmDoMyBest));
        boost::thread th_motion41(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmIni));
        th_motion42.join();
        boost::thread th_motion43(boost::bind(&sp5_control_caller, UNIT_ARM_L, CMD_MOVE_ABS, 8, aArmIni));
        break;
      }
      case 5:
      {
        // Hello
        boost::thread th_motion11(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmHello));
        bool has_completed = th_motion11.timed_join(td5);
        // Thank you
        boost::thread th_motion21(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmThankYou));
        has_completed = th_motion21.timed_join(td1);
        boost::thread th_motion22(boost::bind(&sp5_control_caller, UNIT_LUMBA, CMD_MOVE_REL, 4, aWaistThankYou));
        has_completed = th_motion22.timed_join(td6);
        boost::thread th_motion23(boost::bind(&sp5_control_caller, UNIT_LUMBA, CMD_MOVE_ABS, 3, aWaistIni));
        has_completed = th_motion23.timed_join(td15);
        // Enjoy
        boost::thread th_motion31(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aRArmEnjoy));
        has_completed = th_motion31.timed_join(td15);
        boost::thread th_motion41(boost::bind(&sp5_control_caller, UNIT_ARM_R, CMD_MOVE_ABS, 8, aArmIni));
        // Do my best
        has_completed = th_motion41.timed_join(td5);
        boost::thread th_motion42(boost::bind(&sp5_control_caller, UNIT_ARM_L, CMD_MOVE_ABS, 8, aArmDoMyBest));
        has_completed = th_motion42.timed_join(td7);
        boost::thread th_motion43(boost::bind(&sp5_control_caller, UNIT_ARM_L, CMD_MOVE_ABS, 8, aArmIni));
      }
      default:
      {
        break;
      }
    }
  }
QUIT:
  return 0;
}
