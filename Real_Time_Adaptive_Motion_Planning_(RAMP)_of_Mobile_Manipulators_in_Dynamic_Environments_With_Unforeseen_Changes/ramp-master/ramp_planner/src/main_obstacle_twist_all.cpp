#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ramp_msgs/MotionState.h"
#include "ramp_msgs/ObstacleList.h"
#include "utility.h"

std::vector< std::string > ob_odoms;
std::vector< std::string > ob_vels;
std::vector< ros::Publisher > ob_pubs;
std::vector< double > ob_delays;
std::vector< ros::Duration > dur_delays;
std::vector< ros::Timer > ob_timers;
ros::Time node_start;
Utility utility;

std::vector<ros::Subscriber> sub_ob_ic;
std::vector<bool> ob_ic;

void getObstacleParams(const ros::NodeHandle handle)
{
  if(handle.hasParam("/ramp/obstacle_odoms"))
  {
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
    }
  }


  if(handle.hasParam("/ramp/obstacle_vels"))
  {
    handle.getParam("/ramp/obstacle_vels", ob_vels);
    ROS_INFO("ob_vels.size(): %i", (int)ob_vels.size());
    for(int i=0;i<ob_vels.size();i++)
    {
      ROS_INFO("ob_vels[%i]: %s", i, ob_vels.at(i).c_str());
    }
  }


  if(handle.hasParam("/ramp/obstacle_delays"))
  {
    handle.getParam("/ramp/obstacle_delays", ob_delays);

    for(int i=0;i<ob_delays.size();i++)
    {
      ROS_INFO("ob_delay[%i]: %f", i, ob_delays.at(i));
      dur_delays.push_back(ros::Duration(ob_delays.at(i)));
    }
  }
}




void turn(const int index, const double v, const double w, const double t)
{
  ros::Rate r(15);
  ros::Duration d(t);
  geometry_msgs::Twist twist;
  
  twist.linear.x = v;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = w;
 
  // Drive
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start < d)
  {
    ob_pubs.at(index).publish(twist);
    r.sleep();
  } // end while
}




void SLike(const int index, const double v, const double w, const double t)
{
  double t_each = t/4.f;

  turn(index, v, w, t_each);
  turn(index, v, -w, t_each);
  turn(index, v, w, t_each);
  turn(index, v, -w, t_each);
}


void driveStraight(const int index, const double v, const double t)
{
  //ROS_INFO("In driveStraight");

  ros::Rate r(25);
  ros::Duration d(t);
  geometry_msgs::Twist twist;
  
  twist.linear.x = v;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = 0.f;
 
  int i=0;
  // Drive forward
  ros::Time start = ros::Time::now();
  while(ros::ok() && (ros::Time::now() - start < d) && !ob_ic.at(index))
  {
    /*if(i % 50 == 0)
    {
      ROS_INFO("ob_ic[%i]: %s", index, ob_ic[index] ? "True" : "False");
    }*/
    ob_pubs.at(index).publish(twist);
    r.sleep();
  } // end while
}


void publishToOb(const ros::TimerEvent e, const int index)
{
  ROS_INFO("index: %i", index);
  ROS_INFO("Elapsed time: %f", (ros::Time::now() - node_start).toSec());


  ros::Rate r(5);
  ros::Duration d(1.5);
  geometry_msgs::Twist twist;
  
  twist.linear.x = 0.28f;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = 0.f;

  /*
   * Set motion for Obstacle 1
   */
  if(index == 1)
  {
    
    
    //turn(index, 0.33, 0.33, 10);
    driveStraight(index, 0.33, 10);
    
    //d = ros::Duration(4);

    //twist.angular.z = (index == 1) ? 0.32 : 0.4;
    // Drive forward
    /*ros::Time t = ros::Time::now();
    while(ros::Time::now() - t < d)
    {
      //ob_pubs.at(index).publish(twist);
      r.sleep();
    } // end while*/
  } // end Obstacle 1


  else
  {
 
    // Drive forward
    ros::Time t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self-rotate
    /*twist.linear.x = 0;
    twist.angular.z = 0.44;

    d = ros::Duration(1.5);
    t = ros::Time::now();
    while(ros::Time::now() - t < d)
    {
      pub_twist.publish(twist);

      r.sleep();
    }*/
    


    // Translate+rotate
    twist.linear.x = 0.25;
    //twist.angular.z = -0.68;

    d = ros::Duration(1.5f);
    t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      twist.angular.z = (index == 1) ? -0.6 : -0.32;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self rotate
    twist.linear.x = 0.2;
    //twist.angular.z = 0.4;

    d = ros::Duration(2.5);
    t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      twist.angular.z = (index == 1) ? 0.32 : 0.6;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    twist.linear.x = 0.25;
    //twist.angular.z = 0.4;

    d = ros::Duration(2.5f);
    t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      twist.angular.z = (index == 1) ? -0.6 : -0.4;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self rotate
    twist.linear.x = 0.22;
    //twist.angular.z = 0.4;

    d = ros::Duration(2.);
    t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      twist.angular.z = (index == 1) ? 0.32 : 0.6;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    twist.linear.x = 0.25;
    //twist.angular.z = 0.4;

    d = ros::Duration(3.f);
    t = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - t < d) && !ob_ic.at(index))
    {
      twist.angular.z = (index == 1) ? -0.6 : -0.32;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }

  } // end if turtlebot

  
  // Stop
  twist.linear.x = 0.;
  twist.angular.z = 0.;
  for(int i=0;i<10;i++)
  {
    ob_pubs.at(index).publish(twist);
  }
}


void setDists(const ramp_msgs::ObstacleList obs)
{
  for(uint8_t i=0;i<obs.obstacles.size();i++)
  {
    // Make sure dists is already populated
    
    std::vector<double> ob_loc;
  }
}

void publishToAllObs(const geometry_msgs::Twist twist)
{
  for(uint8_t i=0;i<ob_pubs.size();i++)
  {
    ob_pubs.at(i).publish(twist);
  }
}


void obIcCb(const std_msgs::Bool::ConstPtr data, const int index)
{
  if(index < ob_ic.size())
  {
    ob_ic.at(index) = data->data;
  }
  else
  {
    ob_ic.push_back(data->data);
  }
  //ROS_INFO("In obIcCb:");
  /*for(uint8_t i=0;i<ob_ic.size();i++)
  {
    ROS_INFO("ob_ic[%i]: %s", i, ob_ic.at(i) ? "True" : "False");
  }*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle handle;
  ros::Rate r(5);
  ros::Duration d(3.);
  geometry_msgs::Twist twist;


  getObstacleParams(handle);
  ROS_INFO("Obtained obstacle rosparams, please review and hit enter to continue");
  //std::cin.get();
  
  // Create publishers
  for(uint8_t i=0;i<ob_vels.size();i++)
  {
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>(ob_vels.at(i), 1000);
    ob_pubs.push_back(pub_twist);
  }



  ROS_INFO("Waiting for /ramp/cc_started=true...");

  // Wait for ramp to start moving the robot
  bool cc_started = false;
  while(!cc_started)
  {
    handle.getParam("/ramp/cc_started", cc_started);
    //ROS_INFO("/ramp/cc_started: %s", cc_started ? "True" : "False");
    r.sleep();
    ros::spinOnce();
  }
  //ROS_INFO("Press Enter to begin obstacle movement");

  //std::cin.get();

  node_start = ros::Time::now();
 

  // Start timers
  for(uint8_t i=0;i<ob_delays.size();i++)
  {
    ros::Timer temp = handle.createTimer(ros::Duration(ob_delays.at(i)), boost::bind(publishToOb, _1, i), true, true);
    ob_timers.push_back(temp);
  }

 
 for(uint8_t i=0;i<ob_odoms.size();i++)
 {
   //ROS_INFO("In for, i: %i", i);
   ob_ic.push_back(false);
   std::stringstream topic_str;
   topic_str<<"/obstacle_"<<(int)i<<"/ob_imminent_collision";
   ROS_INFO("topic_str: %s", topic_str.str().c_str());
   ros::Subscriber sub = handle.subscribe<std_msgs::Bool>(topic_str.str(), 10, boost::bind(obIcCb, _1, i));
   sub_ob_ic.push_back(sub);
 }

  ROS_INFO("Starting obstacle motion!");



  ROS_INFO("Obstacle node done");
  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
