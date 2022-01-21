//----------------------------------------------------------
// @file   : robot_virtual_test.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2015.08.05)
// @date   : 2015.08.05
//----------------------------------------------------------

//include
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "string"
#include "geometry_msgs/Twist.h"
//#include "ncurses.h"

#define LINEAR_CHANGE 0.05 /*0.1*/
#define ANGLE_CHANGE 0.2 /*1.8*/

//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_keyop");
  ros::NodeHandle n;
  ros::Publisher key_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);
  geometry_msgs::Twist key_vel;
  
  ros::Rate loop(10);

  double linear_vel = 0.0;
  double angle_vel  = 0.0;

  while(ros::ok())
  {
    char command_c;       //char
    std::string command;  //string
    std::cout << "linear_vel " << linear_vel <<  " angle_vel " << angle_vel << std::endl;

    //input char
    std::cin >> command_c;
    switch(command_c)
    {
      //go forward
      case 'w':
            linear_vel = linear_vel + LINEAR_CHANGE; 
            break;
      //turn left
      case 'a':
            angle_vel = angle_vel + ANGLE_CHANGE; 
            break;
      //turn right
      case 'd':
            angle_vel = angle_vel - ANGLE_CHANGE; 
            break;
      //go back
      case 's':
            linear_vel = linear_vel - LINEAR_CHANGE; 
            break;
      //ros::shutdown
      case 'c':
            ros::shutdown(); 
            break;
      default:
            break;
    }

    //input pub data
    key_vel.linear.x = linear_vel;
    key_vel.angular.z  = angle_vel;
    key_pub.publish(key_vel);

    loop.sleep();
  }  
  return 0;
}
