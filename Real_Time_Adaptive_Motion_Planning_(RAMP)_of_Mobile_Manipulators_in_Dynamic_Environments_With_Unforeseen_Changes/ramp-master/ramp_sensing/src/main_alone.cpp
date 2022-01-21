#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/ObstacleList.h"
#include "obstacle.h"


double rate;
ros::Publisher pub_obj;
std::vector<Obstacle> obs;
ramp_msgs::ObstacleList list;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;


ros::Subscriber sub_ob;
void testObstacleCallback(const nav_msgs::Odometry::ConstPtr& o) 
{
    if(obs.size() == 0)
    {
        Obstacle ob(*o);
        obs.push_back(ob);
        list.obstacles.push_back(ob.buildObstacleMsg());
    }
    else
    {
        obs.at(0).update(*o);
        list.obstacles.at(0) = obs.at(0).buildObstacleMsg();
    }
}





/** Get the other robot's current odometry information and update the dynamicObject */
void updateOtherRobotCb(const nav_msgs::Odometry::ConstPtr& o, const std::string& topic) 
{
  //ROS_INFO("In updateOtherRobotCb");
  //ROS_INFO("topic: %s", topic.c_str());
  int index = topic_index_map[topic];
  //ROS_INFO("index: %i", index);
  
  if(obs.size() < index+1)
  {
    //ROS_INFO("In if obs.size() < index");
    Obstacle temp(*o);
    obs.push_back(temp);
    list.obstacles.push_back(temp.buildObstacleMsg());
  }
  else
  {
    //ROS_INFO("In else");
    obs.at(index).update(*o);
    list.obstacles.at(index) = obs.at(index).buildObstacleMsg();
  }
} //End updateOtherRobotCb




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  //pub_obj.publish(obstacle.buildObstacleMsg());
  pub_obj.publish(list);
} //End sendList




int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ramp_sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  
  /*std::string other_robot_odom;
  handle.getParam("ramp_sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;*/

  /*if(handle.hasParam("/ramp/obstacle_odoms"))
  {
    ROS_INFO("Found rosparam obstacle_odoms");
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
      topic_index_map[ob_odoms.at(i)] = i;
    }
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find obstacle_topics rosparam!");
  }*/

  if(handle.hasParam("/ramp/sensing_cycle_rate"))
  {
    handle.getParam("/ramp/sensing_cycle_rate", rate);
    ROS_INFO("Sensing cycle rate: %f", rate);
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find sensing_cycle_rate rosparam!");
  }

  /*std::vector< ros::Subscriber > subs_obs;
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    ros::Subscriber sub_ob = handle.subscribe<nav_msgs::Odometry>(ob_odoms.at(i), 1, boost::bind(updateOtherRobotCb, _1, ob_odoms.at(i)));
    subs_obs.push_back(sub_ob);
  } // end for*/
  
  sub_ob = handle.subscribe<nav_msgs::Odometry>("obstacle_test", 1, testObstacleCallback);


  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);

  //Timers
  //ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
    ros::Timer timer = handle.createTimer(ros::Duration(1.f / 5.f), publishList);


  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
