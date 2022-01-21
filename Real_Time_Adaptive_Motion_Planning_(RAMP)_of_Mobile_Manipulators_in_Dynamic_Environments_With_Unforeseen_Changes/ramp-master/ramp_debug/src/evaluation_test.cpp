#include <iostream>
#include "ros/ros.h"
#include "utility.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "evaluation_test");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::ServiceClient client_traj = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");
  ros::ServiceClient client_eval = handle.serviceClient<ramp_msgs::EvaluationSrv>("trajectory_evaluation");

  ramp_msgs::RampTrajectory traj1;
  ramp_msgs::RampTrajectory traj2;
  
  /**************************************************/
  /********** Build Paths for trajectories **********/
  /**************************************************/
  
  // ----- Path 1 - straight line to goal ----- //
    ramp_msgs::Path p1;
    ramp_msgs::KnotPoint p1_kp1;
    ramp_msgs::KnotPoint p1_kp2;
    
    // ----- Knot Point 1 positions ----- //
    p1_kp1.motionState.positions.push_back(3.5);
    p1_kp1.motionState.positions.push_back(2.);
    p1_kp1.motionState.positions.push_back(PI);


    // ----- Knot Point 1 velocities ----- //
    p1_kp1.motionState.velocities.push_back(0);
    p1_kp1.motionState.velocities.push_back(0);
    p1_kp1.motionState.velocities.push_back(0);

    
    // ----- Knot Point 2 positions ----- //
    p1_kp2.motionState.positions.push_back(0.);
    p1_kp2.motionState.positions.push_back(3.5);
    p1_kp2.motionState.positions.push_back(PI);


    // ----- Knot Point 2 velocities ----- //
    p1_kp2.motionState.velocities.push_back(0);
    p1_kp2.motionState.velocities.push_back(0);
    p1_kp2.motionState.velocities.push_back(0);
   
    // ----- Push Knot Points onto Path ----- //
    p1.points.push_back(p1_kp1);
    p1.points.push_back(p1_kp2);



  // ----- Path 2 - straight line to goal ----- //
    ramp_msgs::Path p2;
    ramp_msgs::KnotPoint p2_kp1;
    ramp_msgs::KnotPoint p2_kp2;
    ramp_msgs::KnotPoint p2_kp3;
    

    // ----- Knot Point 1 positions ----- //
    p2_kp1.motionState.positions.push_back(3.5);
    p2_kp1.motionState.positions.push_back(2.);
    p2_kp1.motionState.positions.push_back(PI);

    // ----- Knot Point 1 velocities ----- //
    p2_kp1.motionState.velocities.push_back(0);
    p2_kp1.motionState.velocities.push_back(0);
    p2_kp1.motionState.velocities.push_back(0);

    
    // ----- Knot Point 2 positions ----- //
    p2_kp2.motionState.positions.push_back(0.324827);
    p2_kp2.motionState.positions.push_back(3.25643);
    p2_kp2.motionState.positions.push_back(0.864196);

    // ----- Knot Point 2 velocities ----- //
    p2_kp2.motionState.velocities.push_back(0);
    p2_kp2.motionState.velocities.push_back(0);
    p2_kp2.motionState.velocities.push_back(0);


    // ----- Knot Point 3 positions ----- //
    p2_kp3.motionState.positions.push_back(0.);
    p2_kp3.motionState.positions.push_back(3.5);
    p2_kp3.motionState.positions.push_back(PI);

    // ----- Knot Point 3 velocities ----- //
    p2_kp3.motionState.velocities.push_back(0);
    p2_kp3.motionState.velocities.push_back(0);
    p2_kp3.motionState.velocities.push_back(0);

    // ----- Push Knot Points onto Path ----- //
    p2.points.push_back(p2_kp1);
    p2.points.push_back(p2_kp2);
    p2.points.push_back(p2_kp3);


    std::cout<<"\nA to B: "<<u.findAngleFromAToB(p2_kp1.motionState.positions, p2_kp2.motionState.positions);




  /*************************************************/
  /******** Build Requests for trajectories ********/
  /*************************************************/


  ramp_msgs::TrajectoryRequest tr1;
  tr1.path = p1;

  ramp_msgs::TrajectoryRequest tr2;
  tr2.path = p2;

  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr1);
  tr_srv.request.reqs.push_back(tr2);

  /************************************************/
  /************* Get the trajectories *************/
  /************************************************/

  if(client_traj.call(tr_srv)) {
    traj1 = tr_srv.response.resps.at(0).trajectory;
    traj2 = tr_srv.response.resps.at(1).trajectory;
  }


  /***********************************************/
  /********** Evaluate the trajectories **********/
  /***********************************************/
  
  ramp_msgs::EvaluationSrv er_srv1;
  ramp_msgs::EvaluationRequest er1;
  er1.trajectory = traj1;
  
  
  ramp_msgs::EvaluationRequest er2;
  er2.trajectory = traj2;

  er_srv1.request.reqs.push_back(er1);
  er_srv1.request.reqs.push_back(er2);

  client_eval.call(er_srv1);
    


  std::cout<<"\n************** Trajectory 1: ****************";
  std::cout<<"\nFeasible: "<<er_srv1.response.resps.at(0).feasible;
  std::cout<<"\nFitness: "<<er_srv1.response.resps.at(0).fitness;
  std::cout<<"\n*********************************************";
  
  std::cout<<"\n************** Trajectory 2: ****************";
  std::cout<<"\nFeasible: "<<er_srv1.response.resps.at(1).feasible;
  std::cout<<"\nFitness: "<<er_srv1.response.resps.at(1).fitness;
  std::cout<<"\n*********************************************";
  



  std::cout<<"\nExiting Normally\n";
  return 0;
}
