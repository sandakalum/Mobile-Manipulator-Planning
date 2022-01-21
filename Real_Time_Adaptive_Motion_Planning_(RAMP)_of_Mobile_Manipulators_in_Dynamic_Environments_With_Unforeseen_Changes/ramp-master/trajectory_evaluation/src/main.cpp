#include <iostream>
#include <signal.h>
#include "evaluate.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Obstacle.h"

Evaluate ev;
Utility u;
bool received_ob = false;
std::vector<ros::Duration> t_data;
int count_multiple = 0;
int count_single = 0;

/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationSrv::Request& reqs,
                   ramp_msgs::EvaluationSrv::Response& resps) 
{
  int s = reqs.reqs.size();

  if(s > 1)
  {
    count_multiple++;
  }
  else
  {
    count_single++;
  }
  

  ros::Time t_start = ros::Time::now();
  ros::Duration t_elapsed;
  for(uint8_t i=0;i<s;i++)
  {
    //t_start = ros::Time::now();

    ramp_msgs::EvaluationResponse res;
    ROS_INFO("Robot Evaluating trajectory %i: %s", (int)i, u.toString(reqs.reqs[i].trajectory).c_str());
    //////ROS_INFO("Obstacle size: %i", (int)reqs.reqs[i].obstacle_trjs.size());
    ROS_INFO("imminent_collision: %s", reqs.reqs[i].imminent_collision ? "True" : "False");
    ROS_INFO("coll_dist: %f", reqs.reqs[i].coll_dist);
    ROS_INFO("full_eval: %s", reqs.reqs[i].full_eval ? "True" : "False");

    // If more than one point
    if(reqs.reqs.at(i).trajectory.trajectory.points.size() > 1)
    {
      ////////ROS_INFO("More than 1 point, performing evaluation");
      ev.perform(reqs.reqs[i], res);
    }
    // Else we only have one point (goal point)
    else
    {
      res.fitness = 1.f;
      res.feasible = true;
      res.t_firstCollision = ros::Duration(9999.f);
    }

    ROS_INFO("Done evaluating, fitness: %f feasible: %s t_firstCollision: %f", res.fitness, res.feasible ? "True" : "False", res.t_firstCollision.toSec());
    ros::Time t_vec = ros::Time::now();
    resps.resps.push_back(res);
    
  }
  t_elapsed = ros::Time::now() - t_start;
  t_data.push_back(t_elapsed);
  if(t_elapsed.toSec() > 0.01)
  {
    ////ROS_INFO("Long Eval Trajec (total: %i)", (int)reqs.reqs.size());
  }
  ////ROS_INFO("t_elapsed: %f", t_elapsed.toSec());
  return true;
} //End handleRequest


void reportData(int sig)
{

  double avg = ev.t_analy_[0].toSec();
  for(int i=1;i<ev.t_analy_.size();i++)
  {
    avg += ev.t_analy_[i].toSec();
    //////ROS_INFO("t_analy_: %f", ev.t_analy_.at(i).toSec());
  }
  avg /= ev.t_analy_.size();
  //////ROS_INFO("Average t_analy_ duration: %f", avg);
  

  avg = ev.t_numeric_[0].toSec();
  for(int i=1;i<ev.t_numeric_.size();i++)
  {
    avg += ev.t_numeric_[i].toSec();
    ROS_INFO("t_numeric_: %f", ev.t_numeric_.at(i).toSec());
  }
  avg /= ev.t_numeric_.size();
  ROS_INFO("Average t_numeric_ duration: %f", avg);

  

  if(ev.cd_.t_ln.size() > 0)
  {
    avg = ev.cd_.t_ln[0].toSec();
    for(int i=1;i<ev.cd_.t_ln.size();i++)
    {
      avg += ev.cd_.t_ln.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_ln: %f", ev.cd_.t_ln[i].toSec());
    }
    avg /= ev.cd_.t_ln.size();
    //////ROS_INFO("Average ev.cd_.t_ln duration: %f", avg);
  }

  if(ev.cd_.t_bn.size() > 0)
  {
    avg = ev.cd_.t_bn[0].toSec();
    for(int i=1;i<ev.cd_.t_bn.size();i++)
    {
      avg += ev.cd_.t_bn.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_bn: %f", ev.cd_.t_bn[i].toSec());
    }
    avg /= ev.cd_.t_bn.size();
    //////ROS_INFO("Average ev.cd_.t_bn duration: %f", avg);
  }
  

  if(ev.cd_.t_ll.size() > 0)
  {
    avg = ev.cd_.t_ll[0].toSec();
    for(int i=1;i<ev.cd_.t_ll.size();i++)
    {
      avg += ev.cd_.t_ll.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_ll: %f", ev.cd_.t_ll[i].toSec());
    }
    avg /= ev.cd_.t_ll.size();
    //////ROS_INFO("Average ev.cd_.t_ll duration: %f", avg);
  }

  
  if(ev.cd_.t_ll_num.size() > 0)
  {
    avg = ev.cd_.t_ll_num[0].toSec();
    for(int i=1;i<ev.cd_.t_ll_num.size();i++)
    {
      avg += ev.cd_.t_ll_num.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_ll_num: %f", ev.cd_.t_ll_num[i].toSec());
    }
    avg /= ev.cd_.t_ll_num.size();
    //////ROS_INFO("Average ev.cd_.t_ll_num duration: %f", avg);
  }

  if(ev.cd_.t_ln_num.size() > 0)
  {
    avg = ev.cd_.t_ln_num[0].toSec();
    for(int i=1;i<ev.cd_.t_ln_num.size();i++)
    {
      avg += ev.cd_.t_ln_num.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_ln_num: %f", ev.cd_.t_ln_num[i].toSec());
    }
    avg /= ev.cd_.t_ln_num.size();
    //////ROS_INFO("Average ev.cd_.t_ln_num duration: %f", avg);
  }

  if(ev.cd_.t_la.size() > 0)
  {
    avg = ev.cd_.t_la[0].toSec();
    for(int i=1;i<ev.cd_.t_la.size();i++)
    {
      avg += ev.cd_.t_la.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_la: %f", ev.cd_.t_la[i].toSec());
    }
    avg /= ev.cd_.t_la.size();
    //////ROS_INFO("Average ev.cd_.t_la duration: %f", avg);
  }
  

  if(ev.cd_.t_bl.size() > 0)
  {
    avg = ev.cd_.t_bl[0].toSec();
    for(int i=1;i<ev.cd_.t_bl.size();i++)
    {
      avg += ev.cd_.t_bl.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_bl: %f", ev.cd_.t_bl[i].toSec());
    }
    avg /= ev.cd_.t_bl.size();
    //////ROS_INFO("Average ev.cd_.t_bl duration: %f", avg);
  }
  

  if(ev.cd_.t_ba.size() > 0)
  {
    avg = ev.cd_.t_ba[0].toSec();
    for(int i=1;i<ev.cd_.t_ba.size();i++)
    {
      avg += ev.cd_.t_ba.at(i).toSec();
      //////ROS_INFO("ev.cd_.t_ba: %f", ev.cd_.t_ba[i].toSec());
    }
    avg /= ev.cd_.t_ba.size();
    //////ROS_INFO("Average ev.cd_.t_ba duration: %f", avg);
  }


  avg = t_data.at(0).toSec();
  for(int i=1;i<t_data.size();i++)
  {
    avg += t_data.at(i).toSec();
    ROS_INFO("traj_eval: %f", t_data.at(i).toSec());
  }
  avg /= t_data.size();
  ROS_INFO("Average traj_eval duration: %f", avg);

  ////ROS_INFO("# of single evaluations: %i", count_single);
  ////ROS_INFO("# of multiple evaluations: %i", count_multiple);

  //////ROS_INFO("Done reporting");
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_evaluation");
  ros::NodeHandle handle;

  setvbuf(stdout, NULL, _IOLBF, 4096);

  int id;
 
  ros::ServiceServer service    = handle.advertiseService("trajectory_evaluation", handleRequest);

  signal(SIGINT, reportData);
  //cd.pub_population = handle.advertise<ramp_msgs::Population>("/robot_1/population", 1000);

  /** ***Testing*** */


  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();

  //ros::spin();

  printf("\nTrajectory Evaluation exiting normally\n");
  return 0;
}
