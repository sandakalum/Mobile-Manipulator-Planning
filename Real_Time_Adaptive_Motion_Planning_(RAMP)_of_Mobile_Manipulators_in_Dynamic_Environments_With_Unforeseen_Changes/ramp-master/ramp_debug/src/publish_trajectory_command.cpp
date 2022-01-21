#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "ramp_msgs/EvaluationSrv.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierCurve.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");
  ros::ServiceClient client_eval = handle.serviceClient<ramp_msgs::EvaluationSrv>("trajectory_evaluation");


  ramp_msgs::KnotPoint zero;
  zero.motionState.positions.push_back(0.);
  zero.motionState.positions.push_back(0.);
  zero.motionState.positions.push_back(0.785f);
  zero.motionState.velocities.push_back(0.);
  zero.motionState.velocities.push_back(0.);
  zero.motionState.velocities.push_back(0.);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);



  // Build a Path
  ramp_msgs::KnotPoint c1;
  c1.motionState.positions.push_back(0.303921); // 0.70455
  c1.motionState.positions.push_back(1.70294); // 0.4026
  c1.motionState.positions.push_back(-0.767448); // 0.519146
  
  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(0.390587); // 0.70455
  c2.motionState.positions.push_back(1.612194); // 0.4026
  c2.motionState.positions.push_back(-0.819547); // 0.519146


  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(1.70698); // 0.857146
  c3.motionState.positions.push_back(1.53309); // 0.71115
  c3.motionState.positions.push_back(-0.0605828);  // 1.11151


  ramp_msgs::KnotPoint c4;
  c4.motionState.positions.push_back(0.100138);
  c4.motionState.positions.push_back(1.53497);
  c4.motionState.positions.push_back(0.0987241);


  ramp_msgs::KnotPoint c5;
  c5.motionState.positions.push_back(0.111143);
  c5.motionState.positions.push_back(1.52631);
  c5.motionState.positions.push_back(2.47506);

  ramp_msgs::KnotPoint c6;
  c6.motionState.positions.push_back(2.11916);
  c6.motionState.positions.push_back(0.399425);
  c6.motionState.positions.push_back(-0.0710888);

  ramp_msgs::KnotPoint c7;
  c7.motionState.positions.push_back(0.384512);
  c7.motionState.positions.push_back(1.36153);
  c7.motionState.positions.push_back(1.84447);

  ramp_msgs::KnotPoint c8;
  c8.motionState.positions.push_back(0.219097);
  c8.motionState.positions.push_back(0.934402);
  c8.motionState.positions.push_back(1.25361);

  ramp_msgs::KnotPoint c9;
  c9.motionState.positions.push_back(3.5);
  c9.motionState.positions.push_back(3.5);
  c9.motionState.positions.push_back(PI/4);
  
  
  // Velocities
  c1.motionState.velocities.push_back(0.175905);  //.151426
  c1.motionState.velocities.push_back(-0.177644); //-.297903
  c1.motionState.velocities.push_back(0.); //-.118126*/
 
  c2.motionState.velocities.push_back(0.175905);  //.151426
  c2.motionState.velocities.push_back(-0.0106699); //-.297903
  c2.motionState.velocities.push_back(0.); //-.118126

  c3.motionState.velocities.push_back(0.249541);
  c3.motionState.velocities.push_back(-0.0151364);
  c3.motionState.velocities.push_back(0.);

  /*c4.motionState.velocities.push_back(0.23);
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0);*/
 

  c5.motionState.velocities.push_back(0.196492);
  c5.motionState.velocities.push_back(-0.154567);
  c5.motionState.velocities.push_back(0);

  c6.motionState.velocities.push_back(0);
  c6.motionState.velocities.push_back(0);
  c6.motionState.velocities.push_back(0);

  // Accelerations
  /*c1.motionState.accelerations.push_back(0.);
  c1.motionState.accelerations.push_back(0.06665);
  c1.motionState.accelerations.push_back(0.0105176);
  
  c2.motionState.accelerations.push_back(-0.0738498); //.0114877
  c2.motionState.accelerations.push_back(0.06665);  //-.10465
  c2.motionState.accelerations.push_back(0.0430502); //.0746295*/

  c3.motionState.accelerations.push_back(0.);
  c3.motionState.accelerations.push_back(0.);
  c3.motionState.accelerations.push_back(0.);

  /*c4.motionState.accelerations.push_back(0.);
  c4.motionState.accelerations.push_back(0.);
  c4.motionState.accelerations.push_back(0.);*/
  
  c5.motionState.accelerations.push_back(0.); //.0114877
  c5.motionState.accelerations.push_back(0.);  //-.10465
  c5.motionState.accelerations.push_back(0.); //.0746295
  
  ramp_msgs::Path p;
  //p.points.push_back(zero);
  p.points.push_back(c1);
  p.points.push_back(c2);
  p.points.push_back(c3);
  //p.points.push_back(c4);
  //p.points.push_back(c5);
  //p.points.push_back(c6);
  //p.points.push_back(c7);
  //p.points.push_back(c8);
  //p.points.push_back(c9);
  

  /***************************************************/
  /**************** Create Curves ********************/
  /***************************************************/
 
  // Make BezierCurve from Path
  ramp_msgs::BezierCurve bi;
  
  ramp_msgs::MotionState sp0;
  sp0 = p.points.at(0).motionState;
  //sp0 = zero.motionState;
  
  /*sp0.positions.push_back(0.);
  sp0.positions.push_back(1.5);
  sp0.positions.push_back(PI/4);

  sp0.velocities.push_back(0.);
  sp0.velocities.push_back(0.);
  sp0.velocities.push_back(0.);

  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);*/

  
  ramp_msgs::MotionState sp1;
  sp1 = p.points.at(1).motionState;
  //sp1 = p.points.at(0).motionState;
  //sp1 = c2.motionState;

  /*sp1.positions.push_back(1.39389);
  sp1.positions.push_back(1.39292);
  sp1.positions.push_back(0.39449);

  sp1.velocities.push_back(0.);
  sp1.velocities.push_back(0.);
  sp1.velocities.push_back(0);

  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState sp2;
  if(p.points.size() > 2)
    sp2 = p.points.at(2).motionState;
  //sp2 = p.points.at(1).motionState;
  //sp2 = c3.motionState;

  /*sp2.positions.push_back(0.857146);
  sp2.positions.push_back(0.71115);
  sp2.positions.push_back(1.11151);

  sp2.velocities.push_back(0.163205);
  sp2.velocities.push_back(0.33);
  sp2.velocities.push_back(0);

  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);*/

  // *** Push on the Segment points ***
  bi.segmentPoints.push_back(sp0);
  bi.segmentPoints.push_back(sp1);
  bi.segmentPoints.push_back(sp2);


  // Control Points
  
  //ramp_msgs::MotionState cp0 = sp0;
  ramp_msgs::MotionState cp0;
  cp0.positions.push_back(2.21569);
  cp0.positions.push_back(0.336989);
  cp0.positions.push_back(0.146399);

  cp0.velocities.push_back(0.326195);
  cp0.velocities.push_back(0.0499688);
  cp0.velocities.push_back(0);
  
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp1;
  cp1 = sp1;
  //cp1 = p.points.at(1).motionState;
  /*cp1.positions.push_back(1.);
  cp1.positions.push_back(0.5);
  cp1.positions.push_back(-PI/4);

  cp1.velocities.push_back(0.);
  cp1.velocities.push_back(0.);
  cp1.velocities.push_back(0);

  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState cp2;
  cp2.positions.push_back(2.72743);
  cp2.positions.push_back(0.788203);
  cp2.positions.push_back(1.29326);
  
  // *** Push on the Control points ***
  /*bi.controlPoints.push_back(cp0);
  bi.controlPoints.push_back(cp1);
  bi.controlPoints.push_back(cp2);*/


  /** Why 0 initial velocity? **/
  ramp_msgs::MotionState ms_initVA;
  ms_initVA.velocities.push_back(0.);
  ms_initVA.velocities.push_back(0.);
  ms_initVA.velocities.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA;
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.accelerations.push_back(1);
  ms_maxVA.accelerations.push_back(1);



  ramp_msgs::MotionState ms_begin = p.points.at(0).motionState;
  /*ms_begin.positions.push_back(0.391254);
  ms_begin.positions.push_back(0.227599);
  ms_begin.positions.push_back(0.526867);

  ms_begin.velocities.push_back(0.328534);
  ms_begin.velocities.push_back(0.191114);
  ms_begin.velocities.push_back(0.);

  ms_begin.accelerations.push_back(0);
  ms_begin.accelerations.push_back(0.);
  ms_begin.accelerations.push_back(0.);*/

  //bi.ms_begin = ms_begin;
  //bi.ms_initialVA = ms_initVA;
  //bi.ms_maxVA = ms_maxVA;
 

  // u
  /*bi.u_0 = 0.0001;
  bi.u_target = 0.976942;
  bi.u_dot_0 = 0.40706;
  bi.u_dot_max = 0.;
  bi.l = 0.85;*/



  /**************************************************/
  /**************** Curve 0 Done ********************/
  /**************************************************/

  // Curve 1
  ramp_msgs::BezierCurve bi2;


  // Segment points
  ramp_msgs::MotionState sp2_0;
  sp2_0.positions.push_back(0.70455);
  sp2_0.positions.push_back(0.4026);
  sp2_0.positions.push_back(0.519146);

  sp2_0.velocities.push_back(0.33);
  sp2_0.velocities.push_back(0.188571);
  sp2_0.velocities.push_back(0.);

  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);


  ramp_msgs::MotionState sp2_1 = c3.motionState;
  ramp_msgs::MotionState sp2_2 = c4.motionState;

  bi2.segmentPoints.push_back(sp2_0);
  bi2.segmentPoints.push_back(sp2_1);
  bi2.segmentPoints.push_back(sp2_2);


  // Control points
  ramp_msgs::MotionState cp2_0;
  cp2_0.positions.push_back(0.852275);
  cp2_0.positions.push_back(0.7013);
  cp2_0.positions.push_back(1.11151);
  
  cp2_0.velocities.push_back(0.163205);
  cp2_0.velocities.push_back(0.33);
  cp2_0.velocities.push_back(0.);
  
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp2_1 = sp2_1;

  ramp_msgs::MotionState cp2_2;
  cp2_2.positions.push_back(1.3094);
  cp2_2.positions.push_back(1.12376);
  cp2_2.positions.push_back(0.380506);
  
  bi2.controlPoints.push_back(cp2_0);
  bi2.controlPoints.push_back(cp2_1);
  bi2.controlPoints.push_back(cp2_2);

  // u
  bi2.l = 0.5;
  bi2.u_0 = 0.; 
  bi2.u_dot_0 = 0.552394;
  bi2.u_target = 0.959989;


  ramp_msgs::MotionState ms_initVA2;
  ms_initVA2.velocities.push_back(0.163205);
  ms_initVA2.velocities.push_back(0.33);
  ms_initVA2.velocities.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA2;
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.accelerations.push_back(1);
  ms_maxVA2.accelerations.push_back(1);

  bi2.ms_initialVA = ms_initVA2;
  bi2.ms_maxVA = ms_maxVA2;
  
  //bi.ms_begin = p.points.at(0).motionState;


  /**************************************************/
  /**************** Curve 1 Done ********************/
  /**************************************************/





  std::vector<ramp_msgs::BezierCurve> curves;
  curves.push_back(bi);
  //curves.push_back(bi2);
  
  ramp_msgs::TrajectoryRequest tr;
  tr.path = p;
  tr.type = TRANSITION;
  tr.print = true;
  tr.bezierCurves = curves;
  tr.segments = 0;

  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr);

  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  ros::Time t_end, t_start = ros::Time::now();
  // Get and publish trajectory
  if(client_.call(tr_srv)) 
  {
    t_end = ros::Time::now();
    ROS_INFO("Resps size: %i", (int)tr_srv.response.resps.size());
    //std::cout<<"\nSending Trajectory "<<u.toString(tr_srv.response.resps.at(0).trajectory)<<"\n";
    handle.setParam("/ramp/cc_started", true); 
    pub_traj.publish(tr_srv.response.resps.at(0).trajectory);
  }
  else 
  {
    std::cout<<"\nSome error getting trajectory\n";
  }

  tr_srv.response.resps[0].trajectory.t_start = ros::Duration(1.4);
   
  ROS_INFO("Press Enter to evaluate the trajectory");
  std::cin.get();

  ramp_msgs::EvaluationSrv er_srv;
  ramp_msgs::EvaluationRequest er;

  er.trajectory = tr_srv.response.resps.at(0).trajectory;
 
  /*er.trajectory.trajectory.points.resize(17);
  er.trajectory.i_knotPoints.pop_back();
  er.trajectory.i_knotPoints.push_back(er.trajectory.trajectory.points.size()-1);
  er.trajectory.curves.clear();*/

  er.currentTheta = er.trajectory.trajectory.points.at(0).positions.at(2);
  er.imminent_collision = false;
  er.full_eval = true;
  er.consider_trans = true;
  er.trans_possible = false;
  
    
  //std::cout<<"\nEval: Sending Trajectory "<<u.toString(er.trajectory)<<"\n";


  // Build any obstacle trajectories
  ramp_msgs::KnotPoint ob1;
  ob1.motionState.positions.push_back(3.5f); // 0.70455
  ob1.motionState.positions.push_back(3.5f); // 0.4026
  ob1.motionState.positions.push_back(PI/2.f); // 0.519146
 
  ob1.motionState.velocities.push_back(0.); // 0.70455
  ob1.motionState.velocities.push_back(0.); // 0.4026
  ob1.motionState.velocities.push_back(0.); // 0.519146
 
  ramp_msgs::KnotPoint ob2;
  ob2.motionState.positions.push_back(3.5f); // 0.70455
  ob2.motionState.positions.push_back(3.5f); // 0.4026
  ob2.motionState.positions.push_back(0.); // 0.519146
 
  ob2.motionState.velocities.push_back(0.); // 0.70455
  ob2.motionState.velocities.push_back(0.); // 0.4026
  ob2.motionState.velocities.push_back(0.); // 0.519146

  ramp_msgs::Path obp, obp2;
  obp.points.push_back(ob1);
  obp.points.push_back(ob2);

  ramp_msgs::TrajectoryRequest obtr, obtr2;
  obtr.path = obp;
  obtr.type = 3;
 
  obtr2.path = obp2;
  obtr2.type = 3;

  ramp_msgs::TrajectorySrv obtr_srv;
  obtr_srv.request.reqs.push_back(obtr);
  //obtr_srv.request.reqs.push_back(obtr2);

  if(!client_.call(obtr_srv))
  {
    ROS_ERROR("Some error getting obstacle trajectory");
  }

  ramp_msgs::RampTrajectory obt = obtr_srv.response.resps.at(0).trajectory;
  /*ramp_msgs::RampTrajectory obt;
  trajectory_msgs::JointTrajectoryPoint a;
  a.positions = ob1.motionState.positions;
  a.velocities = ob1.motionState.velocities;
  obt.trajectory.points.push_back(a);*/
  //ramp_msgs::RampTrajectory obt2 = obtr_srv.response.resps.at(1).trajectory;

  // Add obstacle trajectory to ER
  er.obstacle_trjs.push_back(obt);
  //er.obstacle_trjs.push_back(obt);
  //er.obstacle_trjs.push_back(obt);
  //er.obstacle_trjs.push_back(obt2);
    
  er_srv.request.reqs.push_back(er);

  /*for(int i=0;i<12;i++)
  {
    er_srv.request.reqs.push_back(er);
  }*/


  // Evaluate trajectory
  if(client_eval.call(er_srv)) 
  {
    std::cout<<"\nEvaluated traj, fitness: "<<er_srv.response.resps.at(0).fitness;
    std::cout<<"\nEvaluated traj, feasible: "<<er_srv.response.resps.at(0).feasible;
    std::cout<<"\nEvaluated traj, t_firstColl: "<<er_srv.response.resps.at(0).t_firstCollision;
  }


  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();


  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop;
  pop.population.push_back(tr_srv.response.resps.at(0).trajectory);
  //pop.population.push_back(obtr_srv.response.resps.at(0).trajectory);
  pop.population.push_back(obt);
  
  pub_pop.publish(pop);
  


  ROS_INFO("Time to get trajec: %f", (t_end-t_start).toSec());


  std::cout<<"\nExiting Normally\n";
  return 0;
}
