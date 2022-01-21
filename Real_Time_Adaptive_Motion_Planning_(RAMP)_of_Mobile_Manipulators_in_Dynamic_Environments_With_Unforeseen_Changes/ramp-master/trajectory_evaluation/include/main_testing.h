/* 
 * File:  trajectory_evaluation_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:03 PM
 */

/* 
 * File:  related to trajectory_evaluation_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:05 PM
 */


#ifndef MAIN_PROCESS_H
#define	MAIN_PROCESS_H

#include <iostream>
#include "ros/ros.h"
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
    //ROS_INFO("Robot Evaluating trajectory %i: %s", (int)i, u.toString(reqs.reqs[i].trajectory).c_str());
    //////ROS_INFO("Obstacle size: %i", (int)reqs.reqs[i].obstacle_trjs.size());
    //ROS_INFO("imminent_collision: %s", reqs.reqs[i].imminent_collision ? "True" : "False");
    //ROS_INFO("coll_dist: %f", reqs.reqs[i].coll_dist);
    //ROS_INFO("full_eval: %s", reqs.reqs[i].full_eval ? "True" : "False");

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

    //ROS_INFO("Done evaluating, fitness: %f feasible: %s t_firstCollision: %f", res.fitness, res.feasible ? "True" : "False", res.t_firstCollision.toSec());
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

#endif	/* MAIN_PROCESS_H */

