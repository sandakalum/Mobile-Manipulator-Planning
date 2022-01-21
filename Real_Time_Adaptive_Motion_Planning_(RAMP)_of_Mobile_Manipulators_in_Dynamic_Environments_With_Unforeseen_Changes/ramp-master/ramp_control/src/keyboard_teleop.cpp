#include <ros/ros.h>
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
//#include "corobot_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"


#define KEYCODE_R 0x65 
#define KEYCODE_L 0x61
#define KEYCODE_U 0x77
#define KEYCODE_D 0x73
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x64
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78

int kfd = 0;
struct termios cooked, raw;
int l_wheel = 0, r_wheel = 0;
//corobot_msgs::MotorCommand motorCommand;
geometry_msgs::Twist twist;


int main(int argc, char** argv) {
  int count=0;

  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle handle;
  //ros::Publisher pub_motors = handle.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 1000);
  //ros::Publisher pub_update = handle.advertise<corobot_msgs::MotorCommand>("update", 1000);
  ros::Publisher pub_twist  = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  char c;
  tcgetattr(kfd, &cooked); 
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  ros::Rate r(25);
  puts("Reading from keyboard");

  while(ros::ok()) {
    if(read(kfd, &c, 1) < 0) {
      perror("read(): ");
      exit(-1);
    }
    printf("\nc: %c - %02x", c, c);

    switch(c) {
      case KEYCODE_U:
        std::cout<<"\nc = UP\n";
        //motorCommand.leftSpeed  = 25;
        //motorCommand.rightSpeed = 25;
        twist.linear.x = 0.33;
        twist.angular.z = 0;
        
        break;

      case KEYCODE_D:
        std::cout<<"\nc = DOWN\n";
        //motorCommand.leftSpeed = -25;
        //motorCommand.rightSpeed = -25;
        twist.linear.x = -0.33;
        twist.angular.z = 0;

        break;

      case KEYCODE_L:
        std::cout<<"\nc = LEFT\n";
        //motorCommand.leftSpeed = -25;
        //motorCommand.rightSpeed = 25;
        twist.angular.z = 3.14159/6;
        twist.linear.x = 0;

        break;

      case KEYCODE_R:
        std::cout<<"\nc = RIGHT\n";
        //motorCommand.leftSpeed = 25;
        //motorCommand.rightSpeed = -25;
        twist.angular.z = -3.14159/6;
        twist.linear.x = 0;

        break;

      case KEYCODE_Q:
        std::cout<<"\nc = UP-LEFT\n";
        twist.linear.x = 0.33;
        twist.angular.z = 0.3;

        break;

      case KEYCODE_E:
        std::cout<<"\nc = UP-RIGHT\n";
        twist.linear.x = 0.33;
        twist.angular.z = -0.3;

        break;

      case KEYCODE_Z:
        std::cout<<"\nc = DOWN-LEFT\n";
        twist.linear.x = -0.33;
        twist.angular.z = 0.3;

        break;

      case KEYCODE_X:
        std::cout<<"\nc = DOWN-RIGHT\n";
        twist.linear.x = -0.33;
        twist.angular.z = -0.3;

        break;

      default:
        std::cout<<"\nc is none!\n";
    }

    //motorCommand.secondsDuration = 1;
    //motorCommand.acceleration = 25;

    //pub_motors.publish(motorCommand);

    ros::Time t = ros::Time::now();
    while((ros::Time::now() - t) < ros::Duration(2)) {
      pub_twist.publish(twist);
      r.sleep();
    }

    tcflush(0, TCIFLUSH);

  }

  std::cout<<"\nExiting Normally\n";
  return 0;
}
