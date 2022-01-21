//include
#include <stdlib.h>
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "string"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#define ANGLE_CHANGE 0.01

//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_keyop");
  ros::NodeHandle n;
  
  ros::Publisher key_pub = n.advertise<geometry_msgs::Twist>("portable1/serial_twist",100);
  ros::Publisher key_pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",100);
  
  geometry_msgs::Twist key_vel;

  double angle_vel  = 0.0;
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "/portable1/base_footprint_combined";
  goal.header.stamp    = ros::Time::now();

  ros::Rate loop(10);
  while(ros::ok())
  {
    char command_c;       //char
    std::string command;  //string
    std::cout << "Input key (a->left / d->right / s->stop / 1 -> goal1 / 2->goal2" << std::endl;
    tf::Quaternion q;

    //input char
    std::cin >> command_c;

    if(command_c != 'f'){

	    ros::param::set("/ninebot_human_follower/move_base_simple/goal", "/move_base_dummy");

	    switch(command_c){
		case 'a':
		    angle_vel = angle_vel + ANGLE_CHANGE; 
		    key_vel.angular.z = angle_vel;
		    key_pub.publish(key_vel);
		    std::cout << "angle_vel " << angle_vel << std::endl;
		    break;
		//turn right
		case 'd':
		    angle_vel = angle_vel - ANGLE_CHANGE; 
		    key_vel.angular.z = angle_vel;
		    key_pub.publish(key_vel);
		    std::cout << "angle_vel " << angle_vel << std::endl;
		   break;
		//stop
		case 's':
		    angle_vel = 0.0;
		    key_vel.angular.z = angle_vel;
		    key_pub.publish(key_vel);
		    std::cout << "angle_vel " << angle_vel << std::endl;
		    break;
		case '1':
		    goal.pose.position.x = -3.0;
		    goal.pose.position.y =  0.0;
		    q.setRPY(0, 0, 3.1415);
		    goal.pose.orientation.x = q.x();
		    goal.pose.orientation.y = q.y();
		    goal.pose.orientation.z = q.z();
		    goal.pose.orientation.w = q.w();
		    key_pub_goal.publish(goal);
		    std::cout << "Set Goal Position " << goal.pose.position.x << " " << goal.pose.position.y << std::endl;
		    break;
		case '2':
		    goal.pose.position.x =  3.0;
		    goal.pose.position.y =  0.0;
		    q.setRPY(0, 0, 0);
		    goal.pose.orientation.x = q.x();
		    goal.pose.orientation.y = q.y();
		    goal.pose.orientation.z = q.z();
		    goal.pose.orientation.w = q.w();
		    key_pub_goal.publish(goal);
		    std::cout << "Set Goal Position " << goal.pose.position.x << " " << goal.pose.position.y << std::endl;
		    break;
		//ros::shutdown
		case 'q':
		    ros::shutdown();
		    break;


		//case 'f':
		    //system("bash follower_launch.sh");
		    //break;


		default:
		    break;
	    }
    }else ros::param::set("/ninebot_human_follower/move_base_simple/goal", "/move_base_simple/goal");

    loop.sleep();
  }  
  return 0;
}
