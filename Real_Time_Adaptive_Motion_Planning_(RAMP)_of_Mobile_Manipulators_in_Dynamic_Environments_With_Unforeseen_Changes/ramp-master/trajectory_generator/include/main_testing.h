/* 
 * File:   related to trajectory_generator_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 9, 2015, 1:13 PM
 */

#ifndef MAIN_PROCESS_H
#define	MAIN_PROCESS_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "mobile_base.h"
#include "prediction.h"
#include "line.h"
#include "circle.h"
#include "ros/ros.h"
#include "bezier_curve.h"
#include "ramp_msgs/Population.h"

Utility utility;


void fixDuplicates(ramp_msgs::TrajectoryRequest& req)
{
  int i=0;
  while(i<req.path.points.size()-1)
  {
    ramp_msgs::MotionState a = req.path.points.at(i).motionState;
    ramp_msgs::MotionState b = req.path.points.at(i+1).motionState;

    if(utility.positionDistance(a.positions, b.positions) < 0.01)
    {
      ROS_WARN("Consecutive duplicate knot points in path:\nPath[%i]:\n%s\nand\nPath[%i]\n%s\nRemoving knot point at index %i", 
          i+1,
          utility.toString(a).c_str(),
          i+1,
          utility.toString(b).c_str(),
          i);
      req.path.points.erase(req.path.points.begin()+i+1);
      i--;
    }

    i++;
  }
}


bool checkGoal(ramp_msgs::TrajectoryRequest req)
{
  ramp_msgs::MotionState a = req.path.points.at(0).motionState;
  ramp_msgs::MotionState b = req.path.points.at(1).motionState;

  if(utility.positionDistance(a.positions, b.positions) < 0.1)
  {
    return true;
  }

  return false;
}



bool requestCallback( ramp_msgs::TrajectorySrv::Request& req,
                      ramp_msgs::TrajectorySrv::Response& res) 
{

  ros::Time t_start = ros::Time::now();
  for(uint8_t i=0;i<req.reqs.size();i++)
  {
    ramp_msgs::TrajectoryRequest treq = req.reqs.at(i); 
    ramp_msgs::TrajectoryResponse tres;
    //ROS_INFO("Trajectory Request Received: %s", utility.toString(treq).c_str());

    /*
     * Check for start == goal
     */
    if(treq.path.points.size() == 2 && checkGoal(treq))
    {
      tres.trajectory.trajectory.points.push_back(utility.getTrajectoryPoint(treq.path.points.at(0).motionState));
      tres.trajectory.i_knotPoints.push_back(0);
      res.resps.push_back(tres);
      continue;
    }

    // Why treq.segments == 1?
    if(treq.type != PREDICTION && treq.type != TRANSITION && (treq.path.points.size() < 3 || treq.segments == 1))
    {
      //ROS_WARN("Changing type to HOLONOMIC");
      treq.type = HOLONOMIC;
      treq.segments++;
    }

    if(treq.type != PREDICTION) 
    {
      fixDuplicates(treq);
      
      MobileBase mobileBase;
      if(!mobileBase.trajectoryRequest(treq, tres))
      {
        res.error = true;
      }

      tres.trajectory.holonomic_path = treq.path;
    }
    else if(treq.path.points.size() > 0) 
    {
      //ROS_INFO("In prediction");
      Prediction prediction;
      prediction.trajectoryRequest(treq, tres);
    }

    if( tres.trajectory.i_knotPoints[0] == tres.trajectory.i_knotPoints[1] )
    {
      //ROS_WARN("First two knot points are equal!");
    }
    //ROS_INFO("Response: %s", utility.toString(tres).c_str());
  
    res.resps.push_back(tres);
  }

  ros::Time t_end = ros::Time::now();
  //ROS_INFO("t_end: %f", (t_end-t_start).toSec());
  return true;
}

#endif	/* MAIN_PROCESS_H */

