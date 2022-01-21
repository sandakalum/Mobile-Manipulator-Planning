#include <stdio.h>
#include <math.h>
#include <float.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <people_msgs/People.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define MARGIN 0.6  // Ninebot Width 546mm, Depth 262mm

bool GetInitPosition = false;
ros::Publisher pub_people, pub_ninebot, pub_vis, pub_vis_array;

struct _Pose{
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

_Pose ninebot_pose;

void peopleCallback(const people_msgs::People &people_in){
  double mindist = DBL_MAX;
  nav_msgs::Odometry ninebot_measured_position;
  people_msgs::People people_out;
  people_out.header.frame_id = "/world_link";


#if 0
  for(int i = 0; i < people_in.people.size(); ++i){
    people_msgs::Person person;
    person = people_in.people.at(i);

    double dist = sqrt(pow(ninebot_pose.x - person.position.x, 2) + pow(ninebot_pose.y - person.position.y, 2));

    if(GetInitPosition && dist < MARGIN){
      if(mindist>dist){
        mindist = dist;
        ninebot_measured_position.header.frame_id = "/world_link";
        ninebot_measured_position.header.stamp = ros::Time::now();
        ninebot_measured_position.pose.pose.position.x = person.position.x;
        ninebot_measured_position.pose.pose.position.y = person.position.y;
        ninebot_measured_position.pose.pose.position.z = 0;
        ninebot_measured_position.pose.pose.orientation.x = 0; 
        ninebot_measured_position.pose.pose.orientation.y = 0;
        ninebot_measured_position.pose.pose.orientation.z = 0; 
        ninebot_measured_position.pose.pose.orientation.w = 1;
      }
    } else {
      people_out.people.push_back(person);
    }
  }

  pub_people.publish(people_out);

  if(mindist<MARGIN) pub_ninebot.publish(ninebot_measured_position);

// 元のコードでうまく行かなかった時用
#else
  
  int ninebot_idx = INT_MAX;

  for(int i = 0; i < people_in.people.size(); ++i){
  people_msgs::Person person;
  person = people_in.people.at(i);

  double dist = sqrt(pow(ninebot_pose.x - person.position.x, 2) + pow(ninebot_pose.y - person.position.y, 2));

    if(GetInitPosition && dist < MARGIN){
      if(mindist > dist){
        mindist = dist;
        ninebot_idx = i;
      }
    }
  }

  for(int j = 0; j < people_in.people.size(); ++j){
    people_msgs::Person person_p;
    person_p = people_in.people.at(j);

    if(GetInitPosition && j == ninebot_idx){
      ninebot_measured_position.header.frame_id = "/world_link";
      ninebot_measured_position.header.stamp = ros::Time::now();
      ninebot_measured_position.pose.pose.position.x = person_p.position.x;
      ninebot_measured_position.pose.pose.position.y = person_p.position.y;
      ninebot_measured_position.pose.pose.position.z = 0;
      ninebot_measured_position.pose.pose.orientation.x = 0; 
      ninebot_measured_position.pose.pose.orientation.y = 0;
      ninebot_measured_position.pose.pose.orientation.z = 0; 
      ninebot_measured_position.pose.pose.orientation.w = 1;
    }else{
      people_out.people.push_back(person_p);
    }
  }

  pub_people.publish(people_out);

  if(mindist < MARGIN) pub_ninebot.publish(ninebot_measured_position);

#endif


/*
  visualization_msgs::MarkerArray markerArray;
  int id = 1;

  for(int i = 0; i < people_out.people.size(); ++i){
    visualization_msgs::Marker marker_object;
    marker_object.header.frame_id = "/world_link";
    marker_object.header.stamp = ros::Time::now();

    marker_object.id = id++;
    marker_object.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_object.action = visualization_msgs::Marker::ADD;

    marker_object.mesh_resource = "package://tms_ss_pot/meshes/WalkingMan4.dae";
    marker_object.mesh_use_embedded_materials = true;
    marker_object.scale.x = 0.025;
    marker_object.scale.y = 0.025;
    marker_object.scale.z = 0.025;
    marker_object.pose.position.x = people_out.people.at(i).position.x;
    marker_object.pose.position.y = people_out.people.at(i).position.y;
    marker_object.pose.position.z = 0.0;
    marker_object.pose.orientation.x = 0;
    marker_object.pose.orientation.y = 0;
    marker_object.pose.orientation.z = 0;
    marker_object.pose.orientation.w = 1;

    marker_object.lifetime = ros::Duration(0.5);

    markerArray.markers.push_back(marker_object);
  }

  pub_vis_array.publish(markerArray);
*/

/*
  if(GetInitPosition){
    visualization_msgs::Marker marker_ninebot;
    marker_ninebot.header.frame_id = "/world_link";
    marker_ninebot.header.stamp = ros::Time::now();

    marker_ninebot.id = 1;
    marker_ninebot.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_ninebot.action = visualization_msgs::Marker::ADD;

    marker_ninebot.mesh_resource = "package://tms_ss_pot/meshes/ninebot_v2.dae";
    marker_ninebot.mesh_use_embedded_materials = true;
    marker_ninebot.scale.x = 0.01;
    marker_ninebot.scale.y = 0.01;
    marker_ninebot.scale.z = 0.01;
    marker_ninebot.pose.position.z = 0.0;
    marker_ninebot.pose.position.x = ninebot_pose.x;
    marker_ninebot.pose.position.y = ninebot_pose.y;
    marker_ninebot.pose.orientation.x = ninebot_pose.qx;
    marker_ninebot.pose.orientation.y = ninebot_pose.qy;
    marker_ninebot.pose.orientation.z = ninebot_pose.qz;
    marker_ninebot.pose.orientation.w = ninebot_pose.qw;

    marker_ninebot.lifetime = ros::Duration(0.5);

    pub_vis.publish(marker_ninebot);
  }

*/

}

void odomCallback(const nav_msgs::Odometry &odom_position){
  ninebot_pose.x = odom_position.pose.pose.position.x;
  ninebot_pose.y = odom_position.pose.pose.position.y;
  ninebot_pose.z = odom_position.pose.pose.position.z;
  ninebot_pose.qx = odom_position.pose.pose.orientation.x;
  ninebot_pose.qy = odom_position.pose.pose.orientation.y;
  ninebot_pose.qz = odom_position.pose.pose.orientation.z;
  ninebot_pose.qw = odom_position.pose.pose.orientation.w;
  GetInitPosition = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ninebot_cropper");
  ros::NodeHandle nh;

  ros::Subscriber people_sub = nh.subscribe("people_p2sen", 10, peopleCallback);
  // std::string cropper_odom_topic_name;
  // nh.param<std::string>("param_name", cropper_odom_topic_name, "/portable1/odom");
  ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 10, odomCallback);

  pub_people = nh.advertise<people_msgs::People>("people_ninebot_cropped", 10);
  pub_ninebot = nh.advertise<nav_msgs::Odometry>("ninebot_measured_pos", 10);
  // pub_vis = nh.advertise< visualization_msgs::Marker >("visualization_ninebot", 1);
  // pub_vis_array = nh.advertise< visualization_msgs::MarkerArray >("visualization_objects", 1);

  ros::spin();
  return 0;
}