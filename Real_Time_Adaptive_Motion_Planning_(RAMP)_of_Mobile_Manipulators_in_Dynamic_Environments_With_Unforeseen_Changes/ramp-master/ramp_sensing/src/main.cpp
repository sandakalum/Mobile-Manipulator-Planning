#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/ObstacleList.h"
#include "obstacle.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "circle_packer.h"
#include <visualization_msgs/MarkerArray.h>


double rate;
ros::Publisher pub_obj, pub_rviz;
std::vector< Obstacle> obs;
ramp_msgs::ObstacleList list;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;
nav_msgs::OccupancyGrid global_grid;

std::vector<tf::Transform> ob_tfs;

size_t prev_size;


void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(std::stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(std::stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}




/** Get the other robot's current odometry information and update the obstacle info */
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
    list.obstacles.push_back(temp.msg_);
  }
  else
  {
    //ROS_INFO("In else");
    obs.at(index).update(*o);
    list.obstacles.at(index) = obs.at(index).msg_;
  }
} //End updateOtherRobotCb




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  //pub_obj.publish(obstacle.buildObstacleMsg());
  pub_obj.publish(list);
} //End sendList


std::vector<visualization_msgs::Marker> convertObsToMarkers()
{
  std::vector<visualization_msgs::Marker> result;
  for(int i=0;i<obs.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "/map";
    marker.ns = "basic_shapes";
    marker.id = i;
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    double x_origin = global_grid.info.origin.position.x;
    double y_origin = global_grid.info.origin.position.y;
    
    //ROS_INFO("Before translate: x_origin: %f y_origin: %f", x_origin, y_origin);

    x_origin *= 20;
    y_origin *= 20;
    
    //ROS_INFO("After translate: x_origin: %f y_origin: %f", x_origin, y_origin);

    // This needs to be generalized to the costmap resolution
    double x = (obs[i].cir_.center.x + x_origin) / 20.f;
    double y = (obs[i].cir_.center.y + y_origin) / 20.f;

    //y *= -1;


    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double radius = obs[i].cir_.radius / 20.f;
    //ROS_INFO("x: %f y: %f radius: %f", x, y, radius);
    
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.1);

    result.push_back(marker);
  }

  return result;
}

void publishMarkers(const ros::TimerEvent& e)
{
  
  // Publish a single Marker
  visualization_msgs::MarkerArray result;

  
  std::vector<visualization_msgs::Marker> markers = convertObsToMarkers();

  //ROS_INFO("Publishing %i markers", (int)markers.size());

  result.markers = markers;

  pub_rviz.publish(result);



  //ROS_INFO("markers.size(): %i", (int)markers.size());
  /*for(int i=0;i<markers.size();i++)
  {
    if(i==0 && markers.size() > 0)
    {
      ROS_INFO("markers.size(): %i", (int)markers.size());
      for(int j=0;j<markers.size();j++)
      {
        ROS_INFO("Circle %i scale: %f, %f", j, markers[j].scale.x, markers[j].scale.y);
      }
    }
    pub_rviz.publish(markers[i]);
  }*/
}




void costmapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  global_grid = *grid;
  //ROS_INFO("Got a new costmap!");
  CirclePacker c(grid);
  std::vector<Circle> cirs = c.go();

  prev_size = obs.size();

  obs.clear();

  for(int i=0;i<cirs.size();i++)
  {
    Obstacle o; 
    o.cir_ = cirs[i];
    obs.push_back(o);
  }
 
  //ROS_INFO("obs.size(): %i", (int)obs.size());
  //ROS_INFO("Leaving Cb");
}


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
  }

  if(handle.hasParam("/ramp/sensing_cycle_rate"))
  {
    handle.getParam("/ramp/sensing_cycle_rate", rate);
    ROS_INFO("Sensing cycle rate: %f", rate);
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find sensing_cycle_rate rosparam!");
  }

  loadObstacleTF();

  // Create subscribers
  std::vector< ros::Subscriber > subs_obs;
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    Obstacle temp;
    temp.T_w_init_ = ob_tfs[i];
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);

    ros::Subscriber sub_ob = handle.subscribe<nav_msgs::Odometry>(ob_odoms.at(i), 1, boost::bind(updateOtherRobotCb, _1, ob_odoms.at(i)));
    subs_obs.push_back(sub_ob);
  } // end for*/

  ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  //Timers
  //ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
  ros::Timer timer_markers = handle.createTimer(ros::Duration(1.f/10.f), publishMarkers);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
