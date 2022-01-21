#include <ros/ros.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "configuration.h"
#include <unistd.h>
using namespace std;

// The configuration of the robot
ramp_msgs::Configuration configuration_;

// Number of thetas to average together
static const int POSE_COUNT_THRESHOLD = 1;

// Vector of theta values
std::vector<float> thetas_;

// Angle at the start - currently does nothing - fix needed?
float angle_at_start = 0.0f;

// Utility instance
Utility u;

// Transformation from odometry to world
//Eigen::Transform<float, 2, Eigen::Affine> T_od_w;
tf::StampedTransform T_od_w;

// Rotation from odometry to world
float theta;



/** Initialize the T_od_w */
void setT_od_w(float x, float y, float orientation) {

  tf::Vector3 pos(x, y, 0);
  T_od_w.setOrigin(pos);
  T_od_w.setRotation(tf::createQuaternionFromYaw(orientation));

  theta = orientation;
}


/** Set configuration_ */
void setConfiguration(float x, float y, float theta) {
  configuration_.K.clear();
  
  configuration_.K.push_back(x);
  configuration_.K.push_back(y);
  configuration_.K.push_back(theta - angle_at_start);
} //End setConfiguration


/** Callback for odometry messages. Calls setConfiguration */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  
  // Push the latest theta
  thetas_.push_back(tf::getYaw(msg->pose.pose.orientation));

  // If we have enough thetas to average,
  if(thetas_.size() == POSE_COUNT_THRESHOLD) {
    
    // Average theta    
    double avg_theta = 0;
    for(unsigned int i=0;i<POSE_COUNT_THRESHOLD;i++) {
      avg_theta += thetas_.at(i) / POSE_COUNT_THRESHOLD;
    }
    
    // Set configuration
    setConfiguration(msg->pose.pose.position.x, msg->pose.pose.position.y, avg_theta);
     
    // Clear vector
    thetas_.clear();
  } //end if
} //End odometryCallback




/*************** Needs to be fixed! getCenter no longer exists! *****************/
void displayConfiguration() {

  if(configuration_.K.size() > 0) {

    // Get Configuration and transform it to world CS
    Configuration c(configuration_);
    c.transformBase(T_od_w);
    
    cout<<"\n\nConfiguration: "<<c.toString();

    //std::vector<float> center = u.getCenter(c.K_, c.K_.at(2));
    //cout<<"\nCenter: ("<<center.at(0)<<", "<<center.at(1)<<")";
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "display_base_location");
  ros::NodeHandle handle;

  ros::Subscriber sub_odometry = handle.subscribe("odometry", 1000, odometryCallback);

  setT_od_w(0, 0, 0);

  for(unsigned int i=0;i<u.standardRanges.size();i++) {
    configuration_.ranges.push_back(u.standardRanges.at(i).buildRangeMsg());
  }


  sleep(1);
  ros::Duration d(0.25);
  while(ros::ok()) {
    displayConfiguration();
    d.sleep();
    ros::spinOnce();
  }

  cout<<"\nExiting Normally\n";
  return 0;
}
