#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "ramp_msgs/EvaluationSrv.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierCurve.h"
#include "ramp_msgs/ModificationRequest.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_move_op");
  ros::NodeHandle handle;

  // Create publishers
  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");
  ros::ServiceClient client_path = handle.serviceClient<ramp_msgs::ModificationRequest>("path_modification");
  ros::ServiceClient client_eval = handle.serviceClient<ramp_msgs::EvaluationSrv>("trajectory_evaluation");

  ramp_msgs::Path p1;
  for(unsigned int i=0;i<4;i++) 
  {
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(i+1);
    kp.motionState.positions.push_back(i+1);
    kp.motionState.positions.push_back(PI/4.);

    p1.points.push_back(kp);
  }
  trajectory_msgs::JointTrajectoryPoint jp;
  jp.positions = p1.points[0].motionState.positions;

  ramp_msgs::RampTrajectory traj;
  traj.trajectory.points.push_back(jp);
  traj.i_knotPoints.push_back(0);
  traj.holonomic_path = p1;

  ROS_INFO("Press Enter to publish path");
  std::cin.get();
  
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  pub_pop.publish(pop);


  // Do the modification
  ramp_msgs::ModificationRequest mod;
  mod.request.paths.push_back(p1);
  mod.request.op = "move";
  mod.request.move_dir = PI/4.;
  mod.request.move_dist = 1.5;

  if(client_path.call(mod))
  {
    ROS_INFO("Path successfully modified");
  }
  else
  {
    ROS_ERROR("Error getting modified path");
  }

  // Get the trajectory for the new path
  ramp_msgs::TrajectoryRequest tr;
  tr.path = mod.response.mod_paths[0];
  tr.type = HYBRID;
  tr.print = true;
  
  std::vector<ramp_msgs::BezierCurve> curves;
  
  // Make BezierCurve from Path
  ramp_msgs::BezierCurve bi;
  
  ramp_msgs::MotionState sp0;
  sp0 = tr.path.points.at(0).motionState;
  ramp_msgs::MotionState sp1;
  sp1 = tr.path.points.at(1).motionState;
  ramp_msgs::MotionState sp2;
  sp2 = tr.path.points.at(2).motionState;
  
  bi.segmentPoints.push_back(sp0);
  bi.segmentPoints.push_back(sp1);
  bi.segmentPoints.push_back(sp2);
 
  curves.push_back(bi);
  tr.bezierCurves = curves;
  tr.segments = 0;

  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr);
  if(!client_.call(tr_srv)) 
  {
    ROS_ERROR("Error getting trajectory");
  }
  else
  {
    ROS_INFO("Got trajectory for modified path");
  }


  // Now evaluate the traj
  ramp_msgs::EvaluationSrv er_srv;
  ramp_msgs::EvaluationRequest er_req;

  er_req.trajectory = tr_srv.response.resps[0].trajectory;
  ROS_INFO("Set trajectory for eval");
  
  ramp_msgs::KnotPoint ob1;
  ob1.motionState.positions.push_back(1.25f); // 0.70455
  ob1.motionState.positions.push_back(1.25f); // 0.4026
  ob1.motionState.positions.push_back(PI/2.f); // 0.519146

  trajectory_msgs::JointTrajectoryPoint ob_jp;
  ob_jp.positions = ob1.motionState.positions;
  
  ramp_msgs::RampTrajectory ob_traj;
  ob_traj.trajectory.points.push_back(ob_jp);

  er_req.obstacle_trjs.push_back(ob_traj);

  ROS_INFO("Setting theta");
  er_req.currentTheta = er_req.trajectory.trajectory.points.at(0).positions.at(2);
  ROS_INFO("Done setting theta");

  er_req.imminent_collision = true;
  er_req.full_eval = true;
  er_req.consider_trans = false;
  er_req.trans_possible = false;

  er_srv.request.reqs.push_back(er_req);
  
  if(client_eval.call(er_srv)) 
  {
    std::cout<<"\nEvaluated traj, fitness: "<<er_srv.response.resps.at(0).fitness;
    std::cout<<"\nEvaluated traj, feasible: "<<er_srv.response.resps.at(0).feasible;
    std::cout<<"\nEvaluated traj, t_firstColl: "<<er_srv.response.resps.at(0).t_firstCollision;
  }

   

  ROS_INFO("Press Enter to publish new path");
  std::cin.get();

  pop.population.clear();
  pop.population.push_back(tr_srv.response.resps[0].trajectory);
  pop.population.push_back(ob_traj);
  pub_pop.publish(pop);

  printf("\nExiting Normally\n");
  return 0;
}
