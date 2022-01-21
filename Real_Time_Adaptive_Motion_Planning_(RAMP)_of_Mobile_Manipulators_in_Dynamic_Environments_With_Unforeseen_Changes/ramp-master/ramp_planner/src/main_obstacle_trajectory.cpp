#include <ros/ros.h>
#include "utility.h"
#include "path.h"
#include "control_handler.h"
#include "trajectory_request_handler.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle handle;

  ros::ServiceClient client = handle.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");
  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("/population", 1000);  
  Utility u;


  MotionState s;
  s.msg_.positions.push_back(1.5);
  s.msg_.positions.push_back(2.f);
  s.msg_.positions.push_back(-3.f*PI/4.f);
  s.msg_.velocities.push_back(-0.23f);
  s.msg_.velocities.push_back(-0.23f);
  s.msg_.velocities.push_back(0.22f);

  MotionState kp;
  kp.msg_.positions.push_back(1.f);
  kp.msg_.positions.push_back(2.f);
  kp.msg_.positions.push_back(0.f);
  kp.msg_.velocities.push_back(0.f);
  kp.msg_.velocities.push_back(0.f);
  kp.msg_.velocities.push_back(0.f);

  MotionState g;
  g.msg_.positions.push_back(3.5f);
  g.msg_.positions.push_back(1.5f);
  g.msg_.positions.push_back(PI);
  g.msg_.velocities.push_back(0.f);
  g.msg_.velocities.push_back(0.f);
  g.msg_.velocities.push_back(0.f);

  KnotPoint kp_s(s);
  KnotPoint kp_g(g);

  Path p(s,g);
  p.addBeforeGoal(kp);

  Path pred;
  pred.msg_.points.push_back(kp_s.buildKnotPointMsg());

  ramp_msgs::TrajectoryRequest tr;
  tr.path = pred.buildPathMsg();
  tr.type = PREDICTION;

  ramp_msgs::BezierCurve curve;
  /*curve.segmentPoints.push_back(p.at(0).motionState_.msg_);
  curve.segmentPoints.push_back(p.at(1).motionState_.msg_);
  curve.segmentPoints.push_back(p.at(2).motionState_.msg_);*/

  tr.bezierCurves.push_back(curve);
  
  ROS_INFO("Press Enter to request and send the trajectory\n");
  std::cin.get();

  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr);

  // Get and publish trajectory
  if(client.call(tr_srv)) 
  {
    ROS_INFO("Got obstacle trajectory!");
  }
  else 
  {
    ROS_WARN("Some error getting obstacle trajectory");
  }

  bool cc_started = false;
  ros::Rate r(10);
  
  while(!cc_started)
  {
    handle.getParam("/ramp/cc_started", cc_started);
    //ROS_INFO("/ramp/cc_started: %s", cc_started ? "True" : "False");
    r.sleep();
    ros::spinOnce();
  }

  ros::Rate rs(7);
  rs.sleep();

  ROS_INFO("Publishing trajectory: %s", u.toString(tr_srv.response.resps.at(0).trajectory).c_str());
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  
  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop;
  pop.population.push_back(tr_srv.response.resps.at(0).trajectory);
  
  pub_pop.publish(pop);

  


  return 0;
}
