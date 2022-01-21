#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "ramp_msgs/Population.h"

ramp_msgs::Population pop;
ramp_msgs::MotionState start_;
Utility u;


void updateCallback(const ramp_msgs::MotionState& msg) {

  start_ = msg;
  
  // Set proper velocity values
  start_.velocities.at(0) = msg.velocities.at(0) * cos(start_.positions.at(2));
  start_.velocities.at(1) = msg.velocities.at(0) * sin(start_.positions.at(2));

  // Set proper acceleration values
  start_.accelerations.at(0) = msg.accelerations.at(0) * cos(start_.positions.at(2));
  start_.accelerations.at(1) = msg.accelerations.at(0) * sin(start_.positions.at(2));


}



int main(int argc, char** argv) {
  ros::init(argc, argv, "interrupt_trajectory");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::Subscriber sub_start = handle.subscribe("update", 1000, &updateCallback);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");


  std::cout<<"\nPress Enter to publish first trajectory\n";
  std::cin.get();
  
  // Build knot points
  ramp_msgs::KnotPoint c1;
  c1.motionState.positions.push_back(0);
  c1.motionState.positions.push_back(0);
  c1.motionState.positions.push_back(0);
  

  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(0.5f);
  c2.motionState.positions.push_back(2.f);
  c2.motionState.positions.push_back(PI/18);
  
  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(1.5);
  c3.motionState.positions.push_back(1.5);
  c3.motionState.positions.push_back(0);

  // Push on velocities
  c1.motionState.velocities.push_back(0);
  c1.motionState.velocities.push_back(0);
  c1.motionState.velocities.push_back(0);

  c2.motionState.velocities.push_back(0);
  c2.motionState.velocities.push_back(0);
  c2.motionState.velocities.push_back(0);
 
  c3.motionState.velocities.push_back(0);
  c3.motionState.velocities.push_back(0);
  c3.motionState.velocities.push_back(0); 
  

  // Build path, get trajectory
  ramp_msgs::Path p;
  p.points.push_back(c1);
  p.points.push_back(c2);
  p.points.push_back(c3);

  ramp_msgs::TrajectoryRequest tr;
  tr.path = p;

  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr);

  // Request and send trajectory
  client_.call(tr_srv);
  ramp_msgs::RampTrajectory trj = tr_srv.response.resps.at(0).trajectory;
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);

  
  // Push trajectory onto population 
  // and publish population
  pop.population.clear();
  pop.population.push_back(trj);
  pub_pop.publish(pop); 
  


  // Wait for 2 seconds
  ros::Duration d(6.0f);
  d.sleep();

  ros::spinOnce();
  
  // Stop the robot from driving
  ramp_msgs::RampTrajectory blank;
  pub_traj.publish(blank);



  // Make trajectory from current ms to a stopped one
  
  // Find the stopped point
  // velocities are m/s, resolution rate is 10 HZ
  // Plan for point to be 1 second away
  // Initialize stopping point to start and then adjust based on v
  std::cout<<"\nstart.positions.size(): "<<start_.positions.size()<<" start_.velocities.size(): "<<start_.velocities.size()<<"\n";
  
  ramp_msgs::MotionState stop = start_;
  for(int i=0;i<3;i++) {
    stop.positions.at(i) += (start_.velocities.at(i) / 10);
    stop.velocities.at(i) = 0;
  }

  // Make knot points for the motion states
  ramp_msgs::KnotPoint ms1;
  ramp_msgs::KnotPoint ms2;
  ms1.motionState = start_;
  ms2.motionState = stop;

  ramp_msgs::Path p2;
  p2.points.push_back(ms1);
  p2.points.push_back(ms2);

  std::cout<<"\nnew path: "<<u.toString(p2);
  
  // Create new trajectory request  
  ramp_msgs::TrajectoryRequest tr2;
  tr2.path = p2;
  ramp_msgs::TrajectorySrv tr_srv2;
  tr_srv2.request.reqs.push_back(tr2);

  // Request trajectory, but don't send yet
  client_.call(tr_srv2);
  ramp_msgs::RampTrajectory trj2 = tr_srv2.response.resps.at(0).trajectory;

  std::cout<<"\nTrajectory 2: "<<u.toString(trj2);
  
  // Push trajectory onto population 
  // and publish population
  pop.population.clear();
  pop.population.push_back(trj2);
  pub_pop.publish(pop); 

  ros::spinOnce();


  // Wait for user to hit Enter, 
  // then publish trajectory
  /*std::cout<<"\nPress Enter to continue\n";
  std::cin.get();*/
  pub_traj.publish(trj2);
  



  // Now create final trajectory from stopped point
  // to goal
  ramp_msgs::Path p3;
  p3.points.push_back(ms2);
  p3.points.push_back(c3);

  // Create new trajectory request  
  ramp_msgs::TrajectoryRequest tr3;
  tr3.path = p3;

  ramp_msgs::TrajectorySrv tr_srv3;
  tr_srv3.request.reqs.push_back(tr3);

  // Request trajectory, but don't send yet
  client_.call(tr_srv3);
  ramp_msgs::RampTrajectory trj3 = tr_srv3.response.resps.at(0).trajectory;
  
  // Push trajectory onto population 
  // and publish population
  pop.population.clear();
  pop.population.push_back(trj3);
  pub_pop.publish(pop); 

  // Wait for user to hit Enter, 
  // then publish trajectory
  /*std::cout<<"\nPress Enter to continue\n";
  std::cin.get();*/
  pub_traj.publish(trj3);

  std::cout<<"\nFinal trajectory: "<<u.toString(trj3);

}
