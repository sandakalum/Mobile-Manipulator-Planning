#include "evaluate.h"

Evaluate::Evaluate() : Q_coll_(10000.f), Q_kine_(100000.f), orientation_infeasible_(0) {}

void Evaluate::perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res)
{
  ////ROS_INFO("In Evaluate::perform()");
  //ros::Time t_start = ros::Time::now();
  
  // Set orientation members
  orientation_.currentTheta_  = req.currentTheta;
  orientation_.theta_at_cc_   = req.theta_cc;

  imminent_collision_ = req.imminent_collision;
  ////ROS_INFO("imminent_collision_: %s", imminent_collision_ ? "True" : "False");


  // Reset orientation_infeasible for new trajectory
  orientation_infeasible_ = false;

  performFeasibility(req);
  ////ROS_INFO("qr_.collision: %s orientation_infeasible_: %s", qr_.collision_ ? "True" : "False", orientation_infeasible_ ? "True" : "False");
  req.trajectory.feasible = !qr_.collision_ && !orientation_infeasible_;
  res.feasible = !qr_.collision_ && !orientation_infeasible_;
  req.trajectory.feasible = res.feasible;
  ////////ROS_INFO("performFeasibility: %f", (ros::Time::now()-t_start).toSec());

  if(qr_.collision_)
  {
    ////ROS_INFO("Not feasible");
    res.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    ////ROS_INFO("Feasible");
    res.t_firstCollision = ros::Duration(9999.f);
  }


  if(req.full_eval)
  {
    //////ROS_INFO("Requesting fitness!");
    performFitness(req.trajectory, req.offset, res.fitness);
  }
  else
  {
    //////ROS_INFO("NOT Requesting fitness!");
  }
  ////////ROS_INFO("performFitness: %f", (ros::Time::now()-t_start).toSec());
}


// Redo this method at some point
// It's modiftying trj AND returning a value
void Evaluate::performFeasibility(ramp_msgs::EvaluationRequest& er) 
{
  ////ROS_INFO("In Evaluate::performFeasibility");
  ros::Time t_start = ros::Time::now();

  // Check collision
  ros::Time t_numeric_start = ros::Time::now();
  cd_.performNum(er.trajectory, er.obstacle_trjs, er.coll_dist, qr_);
  ros::Duration d_numeric   = ros::Time::now() - t_numeric_start;
  t_numeric_.push_back(d_numeric);

  ////ROS_INFO("result.collision: %s", qr_.collision_ ? "True" : "False");
  /*ros::Time t_analy_start = ros::Time::now();
  cd_.perform(er.trajectory, er.obstacle_trjs, qr_);
  ros::Duration d_analy = ros::Time::now() - t_analy_start;
  t_analy_.push_back(d_analy);*/

  ////ROS_INFO("feasible: %s", er.trajectory.feasible ? "True" : "False");
  er.trajectory.feasible            = !qr_.collision_;
  er.trajectory.t_firstCollision    = ros::Duration(qr_.t_firstCollision_);

  ramp_msgs::RampTrajectory* trj = &er.trajectory;
  
  bool moving_forward =     (fabs( sqrt(  (trj->trajectory.points[0].velocities[0]*trj->trajectory.points[0].velocities[0]) +
                                  (trj->trajectory.points[0].velocities[1]*trj->trajectory.points[0].velocities[1]))) > 0)
                ? true 
                : false;

  // Use index 1 in case we are switching from a curve
  // If we are switching from a curve, then we may be stopping to rotate right after the first point
  /*bool moving_on_this_curve = 
      er.trajectory.curves.size()     > 0 && 
    ( er.trajectory.curves.at(0).u_0  > 0.000001 ||
    (utility_.positionDistance(trj->trajectory.points.at(1).positions, 
       er.trajectory.curves.at(0).controlPoints.at(0).positions) < 0.0001) ) 
    ? true
    : false;*/
  //////ROS_INFO("t_moving_on_curve: %f", (ros::Time::now()-t_after).toSec());


  ////ROS_INFO("qr_.collision: %s feasible: %s", qr_.collision_ ? "True" : "False", er.trajectory.feasible ? "True" : "False");
  ////ROS_INFO("moving_forward: %s moving_on_this_curve: %s", moving_forward ? "True" : "False", moving_on_this_curve ? "True" : "False");
  ////ROS_INFO("consider_trans: %s trans_possible: %s", er.consider_trans ? "True" : "False", er.trans_possible ? "True" : "False");
  ////ROS_INFO("getDeltaTheta: %f", fabs(orientation_.getDeltaTheta(er.trajectory)));

  // Check orientation for feasibility
  /*if(moving_forward && fabs(orientation_.getDeltaTheta(er.trajectory)) > 0.25 && !moving_on_this_curve)
  //if(fabs(orientation_.getDeltaTheta(er.trajectory)) > 0.25 && !moving_on_this_curve)
  {
    ////ROS_INFO("In if");
    
    if(er.trajectory.i_knotPoints.size() > 2 && er.trajectory.curves.size() < 2)
    {
      ////ROS_INFO("In inner if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), (int)er.trajectory.curves.size());
      er.trajectory.feasible  = false;
      orientation_infeasible_ = true;
    }
    else if(er.trajectory.i_knotPoints.size() == 2 && er.trajectory.curves.size() < 1)
    {
      ////ROS_INFO("In inner else if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), (int)er.trajectory.curves.size());
      er.trajectory.feasible  = false;
      orientation_infeasible_ = true;
    }
  }*/

  if(!er.imminent_collision && er.consider_trans && !er.trans_possible)
  {
    ////ROS_INFO("In final if statement");
    orientation_infeasible_ = true;
  }
  
  ////////ROS_INFO("performFeasibility time: %f", (ros::Time::now() - t_start).toSec());
}



/** This method computes the fitness of the trajectory_ member */
void Evaluate::performFitness(ramp_msgs::RampTrajectory& trj, const double& offset, double& result) 
{
  //ROS_INFO("In Evaluate::performFitness");
  ros::Time t_start = ros::Time::now();
  
  double cost=0;
  double penalties = 0;

  if(trj.feasible)
  {
    //ROS_INFO("In if(feasible)");
    
    // Get total time to execute trajectory
    double T = trj.trajectory.points.at(trj.trajectory.points.size()-1).time_from_start.toSec();

    /*
     * Trajectory point generation ends at the end of the non-holonomic segment
     * For the remaining segments, estimate the time and orientation change needed to execute them
     */

    // p = last non-holonomic point on trajectory
    trajectory_msgs::JointTrajectoryPoint p = trj.trajectory.points.at(trj.trajectory.points.size()-1);
    //////ROS_INFO("p: %s", utility_.toString(p).c_str());

    // Find knot point index on holonomic path where non-holonomic segment ends
    uint16_t i_end=0;
    for(uint16_t i=0;i<trj.holonomic_path.points.size();i++)
    {
      //////ROS_INFO("i: %i trj.holonomic_path.points.size(): %i", (int)i, (int)trj.holonomic_path.points.size());
      double dist = utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, p.positions);

      ROS_INFO("trj.holonomic_path[%i]: %s", (int)i, utility_.toString(trj.holonomic_path.points[i].motionState).c_str());
      ROS_INFO("dist: %f", dist);
      ROS_INFO("offset: %f", offset);

      // Account for some offset
      if( dist*dist < (0.2 + offset) )
      {
        i_end = i; 
        break;
      }
    } // end for
    
    //ROS_INFO("i_end: %i", (int)i_end);
    //ROS_INFO("trj.holonomic_path.points.size(): %i", (int)trj.holonomic_path.points.size());

    // For each segment in remaining holonomic path,
    // accumulate the distance and orientation change needed for remaining segment
    double dist=0;
    double delta_theta=0;
    double last_theta = p.positions[2];
    for(uint8_t i=i_end;i<trj.holonomic_path.points.size()-1;i++)
    {
      //////ROS_INFO("i: %i", (int)i);
      dist += utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
      
      double theta = utility_.findAngleFromAToB(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
      
      delta_theta += fabs(utility_.findDistanceBetweenAngles(last_theta, theta));
      
      last_theta = theta;
    }
    //ROS_INFO("dist: %f delta_theta: %f", dist, delta_theta);

    double max_v=0.25/2;
    double max_w=PI/8.f;

    // Estimate how long to execute positional and angular displacements based on max velocity
    double estimated_linear   = dist / max_v;
    double estimated_rotation = delta_theta / max_w;

    //ROS_INFO("estimated_linear: %f estimated_rotation: %f", estimated_linear, estimated_rotation);

    T += (estimated_linear + estimated_rotation);

    // Orientation
    double A = orientation_.perform(trj);
    
    //ROS_INFO("T: %f A: %f", T, A);
    cost = T + A;
  }

  else
  {
    ////ROS_INFO("In else(infeasible)"); 
    
    // penalties += orientation_.getPenalty();
    
    ////////ROS_INFO("trj.t_firstColl: %f", trj.t_firstCollision.toSec());

    // Add the Penalty for being infeasible due to collision, at some point i was limiting the time to 10s, but i don't know why
    if(trj.t_firstCollision.toSec() > 0 && trj.t_firstCollision.toSec() < 9998)
    {
      ROS_INFO("In if t_firstCollision: %f", trj.t_firstCollision.toSec());
      ROS_INFO("Collision penalty: %f",(Q_coll_ / trj.t_firstCollision.toSec()));
         
      penalties += (Q_coll_ / trj.t_firstCollision.toSec());
    }
    else
    {
      ROS_INFO("Collision penalty (Full): %f",Q_coll_);
      penalties += Q_coll_;
    }

    // If infeasible due to orientation change
    // If there is imminent collision, do not add this penalty (it's okay to stop and rotate)
    if(orientation_infeasible_ && !imminent_collision_)
    {
      ROS_INFO("In if orientation_infeasible_: %f", orientation_.getDeltaTheta(trj));
      ROS_INFO("Orientation penalty: %f", Q_kine_ * (orientation_.getDeltaTheta(trj) / PI));

      double delta_theta = orientation_.getDeltaTheta(trj) < 0.11 ? 0.1 : orientation_.getDeltaTheta(trj);

      penalties += Q_kine_ * (delta_theta / PI);
    }
  }

  ////ROS_INFO("cost: %f penalties: %f", cost, penalties);
  result = (1. / (cost + penalties));

  ////////ROS_INFO("performFitness time: %f", (ros::Time::now() - t_start).toSec());
  //////ROS_INFO("Exiting Evaluate::performFitness");
} //End performFitness
