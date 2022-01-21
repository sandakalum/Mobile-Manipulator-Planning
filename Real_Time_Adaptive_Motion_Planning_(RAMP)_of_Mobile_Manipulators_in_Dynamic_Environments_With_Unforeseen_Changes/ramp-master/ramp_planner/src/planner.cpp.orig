#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), ob_dists_timer_dur_(0.1), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(1.5f), 
  cc_started_(false), c_pc_(0), transThreshold_(1./50.), num_cc_(0), L_(0.33), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), 
 delta_t_switch_(0.2), stop_(false), moving_on_coll_(false), print_enter_exit_(true)
{
  imminentCollisionCycle_ = ros::Duration(1.f / 20.f);
  generationsPerCC_       = controlCycle_.toSec() / planningCycle_.toSec();

  COLL_DISTS.push_back(0.42);
  COLL_DISTS.push_back(0.4);
}

Planner::~Planner() 
{
  if(h_traj_req_!= 0) 
  {
    delete h_traj_req_;
    h_traj_req_= 0;
  }


  if(h_control_ != 0) 
  {
    delete h_control_;
    h_control_ = 0;
  }

  if(h_eval_req_ != 0) 
  {
    delete h_eval_req_;
    h_eval_req_ = 0;
  }
  
  if(modifier_!= 0) 
  {
    delete modifier_;  
    modifier_= 0;
  }
}





//TODO: Get this from parameters...
/** Transformation matrix of obstacle robot from base frame to world frame*/
void Planner::setOb_T_w_odom() 
{
  
  // Obstacle 1
  tf::Transform temp;
  tf::Vector3 pos(3.5f, 3.5, 0.f);
  temp.setRotation(tf::createQuaternionFromYaw(-3.f*PI/4.f));
  temp.setOrigin(pos);

  ob_T_w_odom_.push_back(temp);

 
  // Obstacle 2
  tf::Vector3 pos_two(0.f, 3.5, 0.f);
  temp.setOrigin(pos_two);
  temp.setRotation(tf::createQuaternionFromYaw(-3.f*PI/4.f));
  
  ob_T_w_odom_.push_back(temp);
} // End setOb_T_w_odom



/** This method determines what type of motion an obstacle has */
const MotionType Planner::findMotionType(const ramp_msgs::Obstacle ob) const 
{
  MotionType result;

  // Find the linear and angular velocities
  tf::Vector3 v_linear;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

  tf::Vector3 v_angular;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

  // Find magnitude of velocity vectors
  float mag_linear_t  = sqrt( tf::tfDot(v_linear, v_linear)   );
  float mag_angular_t = sqrt( tf::tfDot(v_angular, v_angular) );


  // Translation only
  // normally 0.0066 when idle
  if(mag_linear_t >= 0.0001 && mag_angular_t < 0.1) 
  {
    ////ROS_INFO("Obstacle MotionType: Translation");
    result = MT_TRANSLATION;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.1) 
  {
    ////ROS_INFO("Obstacle MotionType: Rotation");
    result = MT_ROTATION;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.1) 
  {
    ////ROS_INFO("Obstacle MotionType: Translation and Rotation");
    result = MT_TRANSLATON_AND_ROTATION;
  }

  // Else, there is no motion
  else 
  {
    ////ROS_INFO("Obstacle MotionType: None");
    result = MT_NONE;
  }

  return result;
} // End findMotionType




/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::RampTrajectory Planner::getPredictedTrajectory(const ramp_msgs::Obstacle ob, const tf::Transform tf) const 
{
  ramp_msgs::RampTrajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);
  

  // Now build a Trajectory Request 
  ramp_msgs::TrajectoryRequest tr;
    tr.path = getObstaclePath(ob, tf, motion_type);
    tr.type = PREDICTION;  // Prediction
  
  ramp_msgs::TrajectorySrv tr_srv;
  tr_srv.request.reqs.push_back(tr);

  // Get trajectory
  if(h_traj_req_->request(tr_srv))
  {
    result = tr_srv.response.resps.at(0).trajectory;
  }

  return result;
} // End getPredictedTrajectory






/** 
 *  This method returns a prediction for the obstacle's path. 
 *  The path is based on 1) the type of motion the obstacle currently has
 *  2) the duration that we should predict the motion for 
 */
const ramp_msgs::Path Planner::getObstaclePath(const ramp_msgs::Obstacle ob, const tf::Transform T_w_odom, const MotionType mt) const 
{
  ramp_msgs::Path result;

  std::vector<ramp_msgs::KnotPoint> path;

  ////ROS_INFO("tf: (%f, %f, %f)", T_w_odom.getOrigin().getX(), T_w_odom.getOrigin().getY(), tf::getYaw(T_w_odom.getRotation()));

  /***********************************************************************
   Create and initialize the first point in the path
   ***********************************************************************/
  ramp_msgs::KnotPoint start;
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.x);
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.y);
  start.motionState.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.x);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.y);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.angular.z);
  
  ////ROS_INFO("start before transform: %s", utility_.toString(start).c_str());

  /** Transform point based on the obstacle's odometry frame */
  // Transform the position
  tf::Vector3 p_st(start.motionState.positions.at(0), start.motionState.positions.at(1), 0); 
  tf::Vector3 p_st_tf = T_w_odom * p_st;

  ////ROS_INFO("p_st: (%f, %f, %f)", p_st.getX(), p_st.getY(), p_st.getZ());
  
  start.motionState.positions.at(0) = p_st_tf.getX();
  start.motionState.positions.at(1) = p_st_tf.getY();
  start.motionState.positions.at(2) = utility_.displaceAngle(
      tf::getYaw(T_w_odom.getRotation()), start.motionState.positions.at(2));
  
  ////ROS_INFO("start position after transform: %s", utility_.toString(start).c_str());
  
  // Transform the velocity
  std::vector<double> zero; zero.push_back(0); zero.push_back(0); 
  double teta = utility_.findAngleFromAToB(zero, start.motionState.positions);
  double phi = start.motionState.positions.at(2);
  double v = start.motionState.velocities.at(0);

  ////ROS_INFO("teta: %f phi: %f v: %f", teta, phi, v);

  start.motionState.velocities.at(0) = v*cos(phi);
  start.motionState.velocities.at(1) = v*sin(phi);

  ////ROS_INFO("start (position and velocity) after transform: %s", utility_.toString(start).c_str());


  if(v < 0) 
  {
    start.motionState.positions.at(2) = utility_.displaceAngle(start.motionState.positions.at(2), PI);
  }
  
  /***********************************************************************
   ***********************************************************************
   ***********************************************************************/

  // Push the first point onto the path
  path.push_back(start);

  if(mt == MT_NONE)
  {
    path.push_back(start);
  }

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(mt == MT_TRANSLATION) 
  {

    // Create the Goal Knotpoint
    ramp_msgs::KnotPoint goal;


    double theta = start.motionState.positions.at(2);
    double delta_x = cos(phi)*ob.odom_t.twist.twist.linear.x;
    double delta_y = sin(phi)*ob.odom_t.twist.twist.linear.x;
    ////ROS_INFO("phi: %f theta: %f delta_x: %f delta_y: %f", phi, theta, delta_x, delta_y);
   

    ros::Duration predictionTime_(12.0f);
    // Get the goal position in the base frame
    tf::Vector3 ob_goal_b(start.motionState.positions.at(0) + (delta_x * predictionTime_.toSec()), 
                          start.motionState.positions.at(1) + (delta_y * predictionTime_.toSec()),
                          0);

    goal.motionState.positions.push_back(ob_goal_b.getX());
    goal.motionState.positions.push_back(ob_goal_b.getY());
    goal.motionState.positions.push_back(start.motionState.positions.at(2));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(0));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(1));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(2));

    ////ROS_INFO("goal: %s", utility_.toString(goal.motionState).c_str());


    // Push goal onto the path
    path.push_back(goal);
  } // end if translation


  //std::cout<<"\nPath: "<<utility_.toString(utility_.getPath(path));
  result = utility_.getPath(path);
  return result; 
}






void Planner::sensingCycleCallback(const ramp_msgs::ObstacleList& msg)
{
  //ROS_INFO("In sensingCycleCallback");
  ////ROS_INFO("msg: %s", utility_.toString(msg).c_str());

  ros::Time start = ros::Time::now();

  Population pop_obs;
  Population copy = population_;

  // For each obstacle, predict its trajectory
  for(uint8_t i=0;i<msg.obstacles.size();i++)
  {
    RampTrajectory ob_temp_trj = getPredictedTrajectory(msg.obstacles.at(i), ob_T_w_odom_.at(i));
    if(ob_trajectory_.size() < i+1)
    {
      ob_trajectory_.push_back(ob_temp_trj);
    }
    else
    {
      ob_trajectory_.at(i) = ob_temp_trj;
    }

    copy.trajectories_.push_back(ob_temp_trj);
    ////ROS_INFO("Time to get obstacle trajectory: %f", (ros::Time::now() - start).toSec());
    ////ROS_INFO("ob_trajectory_: %s", ob_temp_trj.toString().c_str());
  } // end for

  ros::Time s = ros::Time::now();
  //population_  = evaluatePopulation(population_);
  evaluatePopulationOOP();
  
  // Make sure control cycle time is updated
  //controlCycle_ = population_.getEarliestStartTime();
  //controlCycleTimer_.setPeriod(controlCycle_, false);
  

  ////ROS_INFO("Time to evaluate population: %f", (ros::Time::now() - s).toSec());
  ////ROS_INFO("Pop now: %s", population_.toString().c_str());
  
  if(cc_started_)
  {
    ////ROS_INFO("Evaluating movingOn_ in SC");
    //movingOn_       = evaluateTrajectory(movingOn_);
    evaluateTrajectoryOOP(movingOn_, false);
    moving_on_coll_ = !movingOn_.msg_.feasible;
  }

  // Find direction of closest obstacle for "move" operator
  /*uint8_t i_closest=0;
  for(uint8_t i=1;i<msg.obstacles.size();i++)
  {
    if(fabs(utility_.positionDistance(latestUpdate_.msg_.positions, ob_trajectory_.at(i).msg_.trajectory.points.at(0).positions)) < fabs(utility_.positionDistance(latestUpdate_.msg_.positions, ob_trajectory_.at(i_closest).msg_.trajectory.points.at(0).positions)))
    {
      i_closest = i; 
    }
  }

  double dir = utility_.findAngleFromAToB(latestUpdate_.msg_.positions, 
      ob_trajectory_.at(i_closest).msg_.trajectory.points.at(0).positions);
  modifier_->dir_ = -dir;*/
  
  ////ROS_INFO("sensing cycle changing CC period to: %f", controlCycle_.toSec());

  ////ROS_INFO("movingOn_ Feasible: %s", movingOn_.msg_.feasible ? "True" : "False");

  sc_durs_.push_back( ros::Time::now() - start );
  
  sendPopulation(copy);

  //sendPopulation(pop_obs);

  //ROS_INFO("Exiting sensingCycleCallback");
}



const std::vector<Path> Planner::getRandomPaths(const MotionState init, const MotionState goal) 
{
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) 
  {
    
    // Create the path with the start and goal
    Path temp_path = getRandomPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getRandomPaths


const std::vector<Path> Planner::getAdjustedPaths(const MotionState init, const MotionState goal) 
{
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) 
  {
    
    // Create the path with the start and goal
    Path temp_path = getAdjustedPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getAdjustedPaths




/*
 * Generate a path from s to g with completely random intermediate knot points (no constraints imposed)
 */
const Path Planner::getRandomPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  

  // Each trajectory will have a random number of knot points
  // Put a max of 3 knot points for practicality...
  uint8_t n = (rand() % 4)+1;

  // Create n knot points 
  for(uint8_t i=0;i<n;i++) 
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Push on velocity values?
    // 
    
    result.addBeforeGoal(ms_temp);
  }

  return result;
} // End getRandomPath



/*
 * Return true if the MotionState ms satisfies constraints on Knot Points to be added to Path p
 */
const bool Planner::validKPForPath(const MotionState ms, const Path p) const
{
  bool result=true;

  // Check that ms_temp has a distance > L from all other knot points
  for(uint8_t i=0;i<p.size();i++)
  {
    if( sqrt( pow(ms.msg_.positions.at(0) - p.at(i).motionState_.msg_.positions.at(0), 2) +
              pow(ms.msg_.positions.at(1) - p.at(i).motionState_.msg_.positions.at(1), 2) ) < L_)
    {
      result = false;
      i = p.size();
    }
  }

  return result;
} // End validForKP



/*
 * Generate a path from s to g with intermediate knot points that are constrained
 */
const Path Planner::getAdjustedPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  
  // Each trajectory will have a random number of knot points
  // Put a max of 4 knot points for practicality...
  uint8_t n = (rand() % 4)+1;

  while(result.size() < (n+2))
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Check that it satisfies any constraints on KPs
    if(validKPForPath(ms_temp, result))
    {
      result.addBeforeGoal(ms_temp);
    }
  } // end while

  return result;
} // End getAdjustedPath



const Population Planner::getPopulation( const MotionState init, const MotionState goal, const bool random)
{
  Population result;

  // Set the size
  result.maxSize_ = populationSize_;
  result.type_    = pop_type_;

  // Get some random paths
  std::vector<Path> paths = random ?  getRandomPaths  (init, goal)  : 
                                      getAdjustedPaths(init, goal)  ;

  // Get trajectories for the paths
  std::vector<RampTrajectory> trajecs = getTrajectories(paths);

  // Add each trajectory to the population
  // Use add over replaceAll in case of sub-populations
  for(uint8_t i=0;i<trajecs.size();i++) 
  {
    result.add(trajecs.at(i));
  }

  // Create sub-pops if enabled
  if(subPopulations_) 
  {
    result.createSubPopulations();
  }

  // Evaluate the population 
  ////ROS_INFO("Calling evaluatePopulation in getPopulation");
  //result = evaluatePopulation(result);
  //population_ = result;
  //evaluatePopulationOOP();

  //////ROS_INFO("Exiting Planner::getRandomPopulation");
  return result;
} // End getPopulation




/*
 * This method assumes the robot cannot complete a curve within a single control cycle!!
 * If the robot DOES complete a curve within a single cc, it will still report the curve as being
 * unfinished which will likely result in the robot turning around to go back to the first control point
 */
const uint8_t Planner::getIndexStartPathAdapting(const RampTrajectory t) const 
{
  /*//ROS_INFO("In Planner::getIndexStartPathAdapting");
  //ROS_INFO("t transTraj.size(): %i", (int)t.transitionTraj_.trajectory.points.size());
  //ROS_INFO("# of curves: %i", (int)t.msg_.curves.size());*/
  uint8_t result;
  bool    has_curve = t.msg_.curves.size() > 0;

  if(t.transitionTraj_.trajectory.points.size() > 0) 
  {
    ////ROS_INFO("In t.transitionTraj_.trajectory.points.size() > 0");
    result = t.transitionTraj_.i_knotPoints.size();
  }
  else if(t.msg_.curves.size() > 1 && t.transitionTraj_.trajectory.points.size() == 0)
  {
    result = 3;
  }
  else if(has_curve && t.msg_.curves.at(0).u_0 == 0) 
  {
    result = 2;
  }
  else {
    result = 1;
  }

  // If the first part is just self-rotation to correct orientation,
  // add 1 to result
  /*if(t.msg_.i_knotPoints.size() > 1 && utility_.positionDistance( 
                                        t.msg_.trajectory.points.at( t.msg_.i_knotPoints.at(1)).positions,
                                        t.msg_.trajectory.points.at( t.msg_.i_knotPoints.at(0)).positions) 
      < 0.001)
  {
    //ROS_WARN("Adding 1 to result because first two position are the same, indicating a rotation to satisfy orientation");
    result++;
  }*/

  ////ROS_INFO("getIndexStartPathAdapting returning: %i", result);
  return result;
}




const uint8_t Planner::getNumThrowawayPoints(const RampTrajectory traj, const ros::Duration dur) const {
  uint8_t result = 1;

  // For each knot point,
  // Start at 2 because that is the end of the first bezier curve
  for(uint8_t i_kp=getIndexStartPathAdapting(traj);
      i_kp<traj.msg_.i_knotPoints.size();
      i_kp++) 
  {
    ////ROS_INFO("i_kp: %i", (int)i_kp);
    
    // Only adapt the best trajectory
    // TODO: Make this method not do a loop
    if(traj.equals(population_.getBest()))
    {
      // Get the knot point 
      trajectory_msgs::JointTrajectoryPoint point = traj.msg_.trajectory.points.at( 
                                                      traj.msg_.i_knotPoints.at(i_kp));
      ////ROS_INFO("point: %s", utility_.toString(point).c_str());

      // Compare the durations
      if( (dur > point.time_from_start) || 
          (fabs(dur.toSec() - point.time_from_start.toSec()) < 0.0001) ) 
      {
        ////ROS_INFO("Past KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
        result++;
      }
      else {
        ////ROS_INFO("Behind KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
        break;
      }
    } // end if best trajectory
    else
    {
      ////ROS_INFO("Not best trajectory in transition population, only removing first knot point (startPlanning_)");
    }
  } // end for


  return result;
}

/** 
 * This method updates all the paths with the current configuration 
 * For each knot point in a path, it will remove the knot point if
 * its time_from_start is <= the Duration argument
 * */
const std::vector<Path> Planner::adaptPaths(const Population pop, const MotionState start, ros::Duration dur) const 
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::adaptPaths");
  }

  ////ROS_INFO("pop.paths.size(): %i", (int)pop.paths_.size());
  ////ROS_INFO("dur.toSec(): %f", dur.toSec());
  std::vector<Path> result;

  // Check that time has passed
  if(dur.toSec() > 0) 
  {


    // For each trajectory
    for(uint8_t i=0;i<pop.size();i++) {
      //ROS_INFO("Path: %s", pop.paths_.at(i).toString().c_str());
      //ROS_INFO("Get Path: %s", pop.get(i).getNonHolonomicPath().toString().c_str());
      Path temp = pop.paths_.at(i);

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=getNumThrowawayPoints(pop.get(i), dur);
      ////ROS_INFO("throwaway: %i", (int)throwaway);

      
      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= pop.paths_.at(i).size() ) 
      { 
        //////ROS_INFO("Decrementing throwaway");
        throwaway = pop.paths_.at(i).size()-1;
      }

      // Erase the amount of throwaway points (points we have already passed)
      temp.msg_.points.erase( 
          temp.msg_.points.begin(), 
          temp.msg_.points.begin()+throwaway );

      KnotPoint kp(start);

      // Insert the new starting configuration
      temp.msg_.points.insert( temp.msg_.points.begin(), kp.buildKnotPointMsg() );

      // Set start_ to be the new starting configuration of the path
      temp.start_ = start;
      //ROS_INFO("After adapting Path: %s", temp.toString().c_str());

      result.push_back(temp);
    } // end outer for
  } // end if dur > 0

  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting adaptPaths");
  }

  return result;
} // End adaptPaths


void Planner::adaptPathsOOP(const MotionState& ms, const ros::Duration& d, std::vector<Path>& result)
{
  if(print_enter_exit_)
  {
    ROS_INFO("In Planner::adaptPathsOOP");
  }

  result.clear();

  ////ROS_INFO("pop.paths.size(): %i", (int)pop.paths_.size());
  ////ROS_INFO("dur.toSec(): %f", dur.toSec());

  // Check that time has passed
  if(d.toSec() > 0) 
  {

    // For each trajectory
    for(uint8_t i=0;i<population_.size();i++) {
      //ROS_INFO("Path: %s", population_.paths_.at(i).toString().c_str());
      //ROS_INFO("Get Path: %s", population_.get(i).getNonHolonomicPath().toString().c_str());
      Path temp = population_.paths_.at(i);

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=getNumThrowawayPoints(population_.get(i), d);
      ROS_INFO("throwaway: %i", (int)throwaway);

      
      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= temp.size() ) 
      { 
        //////ROS_INFO("Decrementing throwaway");
        throwaway = temp.size()-1;
      }

      // Erase the amount of throwaway points (points we have already passed)
      temp.msg_.points.erase( 
          temp.msg_.points.begin(), 
          temp.msg_.points.begin()+throwaway );

      KnotPoint kp(ms);

      // Insert the new starting configuration
      temp.msg_.points.insert( temp.msg_.points.begin(), kp.buildKnotPointMsg() );

      // Set start_ to be the new starting configuration of the path
      temp.start_ = ms;
      ROS_INFO("After adapting Path: %s", temp.toString().c_str());

      result.push_back(temp);
    } // end outer for
  } // end if dur > 0

  if(print_enter_exit_)
  {
    ROS_INFO("Exiting adaptPathsOOP");
  }
} // End adaptPathsOOP





// 1 if before curve, 2 if on curve, 3 if past curve 
// TODO: Check for the 2nd segment as well?
const int Planner::estimateIfOnCurve(const MotionState ms, const ramp_msgs::BezierCurve curve) const {
  ////ROS_INFO("In estimateIfOnCurve");
  ////ROS_INFO("ms: %s", ms.toString().c_str());
  ////ROS_INFO("curve: %s", utility_.toString(curve).c_str());

  double x = ms.msg_.positions.at(0);
  double y = ms.msg_.positions.at(1);

  bool xSlope     = (curve.segmentPoints.at(1).positions.at(0) - curve.segmentPoints.at(0).positions.at(0) > 0);
  bool xSlopeTwo  = (curve.segmentPoints.at(2).positions.at(0) - curve.segmentPoints.at(1).positions.at(0) > 0);
  bool ySlope     = (curve.segmentPoints.at(1).positions.at(1) - curve.segmentPoints.at(0).positions.at(1) > 0);
  bool ySlopeTwo  = (curve.segmentPoints.at(2).positions.at(1) - curve.segmentPoints.at(1).positions.at(1) > 0);
  
  bool xSegOne =  xSlope ?  (x >= curve.controlPoints.at(0).positions.at(0)) &&
                            (x <= curve.controlPoints.at(1).positions.at(0)) :
                            (x <= curve.controlPoints.at(0).positions.at(0)) &&
                            (x >= curve.controlPoints.at(1).positions.at(0));

  bool xSegTwo =  xSlopeTwo ?   (x >= curve.controlPoints.at(1).positions.at(0)) &&
                                (x <= curve.controlPoints.at(2).positions.at(0)) :
                                (x <= curve.controlPoints.at(1).positions.at(0)) &&
                                (x >= curve.controlPoints.at(2).positions.at(0));

  
  bool ySegOne =  ySlope ?  (y >= curve.controlPoints.at(0).positions.at(1)) &&
                            (y <= curve.controlPoints.at(1).positions.at(1)) :
                            (y <= curve.controlPoints.at(0).positions.at(1)) &&
                            (y >= curve.controlPoints.at(1).positions.at(1));

  bool ySegTwo =  ySlopeTwo ?   (y >= curve.controlPoints.at(1).positions.at(1)) &&
                                (y <= curve.controlPoints.at(2).positions.at(1)) :
                                (y <= curve.controlPoints.at(1).positions.at(1)) &&
                                (y >= curve.controlPoints.at(2).positions.at(1));


  //////ROS_INFO("xSlope: %s xSlopeTwo: %s ySlope: %s ySlopeTwo: %s", xSlope ? "True" : "False", xSlopeTwo ? "True" : "False", ySlope ? "True" : "False", ySlopeTwo ? "True" : "False"); 
  //////ROS_INFO("xSegOne: %s xSegTwo: %s ySegOne: %s ySegTwo: %s", xSegOne ? "True" : "False", xSegTwo ? "True" : "False", ySegOne ? "True" : "False", ySegTwo ? "True" : "False");

  bool xGood = (xSegOne || xSegTwo);
  bool yGood = (ySegOne || ySegTwo);

  if(xGood && yGood) {
    //////ROS_INFO("Returning 2 (on curve)");
    return 2;
  }
  
  // Check if past segment 1
  bool xPastOne = xSlope ?  x > curve.controlPoints.at(1).positions.at(0) :
                            x < curve.controlPoints.at(1).positions.at(0) ; 
  // Check if past segment 1
  bool yPastOne = ySlope ?  y > curve.controlPoints.at(1).positions.at(1) :
                            y < curve.controlPoints.at(1).positions.at(1) ; 

  //////ROS_INFO("xPastOne: %s yPastOne: %s", xPastOne ? "True" : "False", yPastOne ? "True" : "False");
  // If past segment 1, check if past segment 2
  if(xPastOne && yPastOne)
  { 
  
    bool xPastTwo = xSlopeTwo ?   x > curve.controlPoints.at(2).positions.at(0) :
                                  x < curve.controlPoints.at(2).positions.at(0) ; 
    bool yPastTwo = ySlopeTwo ?   y > curve.controlPoints.at(2).positions.at(1) :
                                  y < curve.controlPoints.at(2).positions.at(1) ; 

    //////ROS_INFO("xPastTwo: %s yPastTwo: %s", xPastTwo ? "True" : "False", yPastTwo ? "True" : "False");
    if(xPastTwo || yPastTwo)
    {
      //////ROS_INFO("Returning 3 (after curve)");
      return 3;
    }
  } // end if past segment 1

  // Else, robot has not reached curve, return 1
  ////ROS_INFO("Returning 1");
  return 1;
}



const ramp_msgs::BezierCurve Planner::handleCurveEnd(const RampTrajectory traj) const 
{
  ramp_msgs::BezierCurve result;

  // If there are 2 curves, set the next one up 
  // Currently, the adaptive control cycles always end at
  // the start of the next curve so we can safely assume the position should be 0
  // If the control cycles will ever occur in the middle of a 2nd curve, we will have to calculate u_0
  if(traj.msg_.curves.size() > 1) 
  {
    result = traj.msg_.curves.at(1);
    //////ROS_INFO("Curve 0 has ended, new curve: %s", utility_.toString(result).c_str());
    if(estimateIfOnCurve(startPlanning_, result) == 2) {
      //////ROS_INFO("Adding .000001");
      result.u_0 += 0.000001;
      result.ms_begin = startPlanning_.msg_;
    }
    else 
    {
      result.u_0 += 0.;
    }
  }

  // Else if there was only 1 curve, check the path and set the segment points for the next one
  else
  {
    if(traj.msg_.holonomic_path.points.size() > 3)
    {
      //////ROS_INFO("traj.path: %s", traj.msg_.holonomic_path.toString().c_str());
      if(traj.msg_.holonomic_path.points.size() == 3)
      {

      } // end if size==3
      if(traj.msg_.holonomic_path.points.size() < 4)
      {
        ////ROS_ERROR("traj.path.size(): %i", (int)traj.msg_.holonomic_path.points.size());
      }
      result.segmentPoints.push_back(traj.msg_.holonomic_path.points.at(1).motionState);
      result.segmentPoints.push_back(traj.msg_.holonomic_path.points.at(2).motionState);
      result.segmentPoints.push_back(traj.msg_.holonomic_path.points.at(3).motionState);
    } // end if path.size() > 2
  } // end if only one curve

  return result;
} // End handleCurveEnd



const double Planner::updateCurvePos(const RampTrajectory& traj, const ros::Duration& d) const 
{
  if(print_enter_exit_)
  {
    ROS_INFO("In Planner::updateCurvePos");
    //ROS_INFO("d: %f", d.toSec());
  }

  
  ramp_msgs::BezierCurve curve = traj.msg_.curves.at(0); 
  double result = curve.u_0;

  // if u_0==0 then estimateIfOnCurve returned 2 - already on curve
  if(curve.u_0 < 0.00001) 
  {
    //////ROS_INFO("In if curve.u_0==0");

    // Get the time at the start of the curve
    double t_s0 = traj.msg_.trajectory.points.at(
        traj.msg_.i_knotPoints.at(1)).time_from_start.toSec();
    //////ROS_INFO("t_s0: %f", t_s0);

    // t = the time spent moving on the curve
    double t = d.toSec() - t_s0;
    //////ROS_INFO("t: %f", t);
    
    // Previously was subtracting 1 from index. Not sure why that worked, but keep in mind if future issues arise
    // Check if index >= size because of rounding errors
    int index = floor(t*10)+1 >= curve.u_values.size() ? floor(t*10) : floor(t*10)+1;
    //////ROS_INFO("index: %i u_values.size: %i", index, (int)curve.u_values.size());
    if(index < curve.u_values.size())
    {
      //////ROS_INFO("u_values[%i]: %f", index, curve.u_values.at(index));
    }

    if(t < 0.0001)
    {
      result = 0.0001;
    }
    else if(index >= curve.u_values.size())
    {
      //////ROS_INFO("index: %i curve.u_values.size(): %i, setting result to 1.1", index, (int)curve.u_values.size());
      result = 1.1;
    }
    else
    {

      //////ROS_INFO("t: %f Adding %f", t, (t*curve.u_dot_0));
      
      //result += t * curve.u_dot_0;
      result = curve.u_values.at(index);
    }
  } // end if already on curve

  // Else, u_0 > 0, simply add to u_0
  else 
  {
    //////ROS_INFO("Else curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
    result += curve.u_dot_0 * d.toSec();
  }

  if(print_enter_exit_)
  {
    ROS_INFO("Exiting Planner::updateCurvePos");
  }

  return result;
} // End updateCurvePos




void Planner::adaptCurvesOOP(const MotionState& ms, const ros::Duration& d, std::vector<ramp_msgs::BezierCurve>& result)
{
  if(print_enter_exit_)
  {
    ROS_INFO("In Planner::adaptCurvesOOP");
  }

  ramp_msgs::BezierCurve blank;

  // Go through each trajectory 
  for(uint16_t i=0;i<population_.size();i++) 
  {
    ////ROS_INFO("Trajectory %i", (int)i);
    
    // If the trajectory has a curve
    // Don't check for best trajec here b/c we want to push on the same curve if we haven't moved on it, not a blank 
    // curve
    if(population_.get(i).msg_.curves.size() > 0) 
    {
      ////////ROS_INFO("In if trajectory has curve");

      // Set curve
      ramp_msgs::BezierCurve curve = population_.get(i).msg_.curves.size() > 1 ? population_.get(i).msg_.curves.at(1) :
                                                                        population_.get(i).msg_.curves.at(0) ;
      ////ROS_INFO("Set curve to: %s", utility_.toString(curve).c_str());

      ////////ROS_INFO("population_.getBestIndex: %i", (int)population_.calcBestIndex());
      // If moving on this curve, update u
      if( i == population_.calcBestIndex() && 
            (curve.u_0 > 0. ||
             estimateIfOnCurve(ms, curve) == 2))
      {
        ////ROS_INFO("Moving on this curve");

        // Get the new u_0 value
        curve.u_0 = updateCurvePos(population_.get(i), d);

        // Set the new ms_begin
        curve.ms_begin = ms.msg_;
      }  //end if moving on curve
      else if(i != population_.calcBestIndex())
      {
        ////ROS_INFO("Not moving on curve, erase it and start with new segment points");
        curve = blank;
      }
      else
      {
        ////ROS_INFO("Curve is for best trajectory, but not yet moving on curve");
      }


      /* Separate checking if on the curve and if done with curve
           because we could be done before ever incrementing u_0 */
      // Check if done with current curve
      if( i == population_.calcBestIndex() && (curve.u_0 > curve.u_target || estimateIfOnCurve(ms, curve) == 3) )
      {
        ////ROS_INFO("Done with curve, u_0: %f", curve.u_0);
        curve = handleCurveEnd(population_.get(i));
      } // end if done with 1st curve
      else
      {
        ////ROS_INFO("Not done with curve");
      }

      ////ROS_INFO("Curve after adapting: %s", utility_.toString(curve).c_str());
      result.push_back(curve);
    } // end if trajectory has curve

    // Else if there is no curve, push on a blank one
    else 
    {
      ////ROS_INFO("No curve");
      result.push_back(blank);
    } // end else no curve
  } // end for

  if(print_enter_exit_)
  {
    ROS_INFO("Exiting Planner::adaptCurvesOOP");
  }
} // End adaptCurvesOOP



/** Updates the curve u_0 and ms_begin */
// TODO: Only need to update the bestTrajec's curve
const std::vector<ramp_msgs::BezierCurve> Planner::adaptCurves(const Population pop, const MotionState ms, const ros::Duration d) const 
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::adaptCurves");
  }

  std::vector<ramp_msgs::BezierCurve> result;
  ramp_msgs::BezierCurve blank;

  // Go through each trajectory 
  for(uint16_t i=0;i<pop.size();i++) 
  {
    ////ROS_INFO("Trajectory %i", (int)i);
    
    // If the trajectory has a curve
    // Don't check for best trajec here b/c we want to push on the same curve if we haven't moved on it, not a blank 
    // curve
    if(pop.get(i).msg_.curves.size() > 0) 
    {
      ////////ROS_INFO("In if trajectory has curve");

      // Set curve
      ramp_msgs::BezierCurve curve = pop.get(i).msg_.curves.size() > 1 ? pop.get(i).msg_.curves.at(1) :
                                                                        pop.get(i).msg_.curves.at(0) ;
      ////ROS_INFO("Set curve to: %s", utility_.toString(curve).c_str());

      ////////ROS_INFO("pop.getBestIndex: %i", (int)pop.calcBestIndex());
      // If moving on this curve, update u
      if( i == pop.calcBestIndex() && 
            (curve.u_0 > 0. ||
             estimateIfOnCurve(ms, curve) == 2))
      {
        ////ROS_INFO("Moving on this curve");

        // Get the new u_0 value
        curve.u_0 = updateCurvePos(pop.get(i), d);

        // Set the new ms_begin
        curve.ms_begin = ms.msg_;
      }  //end if moving on curve
      else if(i != pop.calcBestIndex())
      {
        ////ROS_INFO("Not moving on curve, erase it and start with new segment points");
        curve = blank;
      }
      else
      {
        ////ROS_INFO("Curve is for best trajectory, but not yet moving on curve");
      }


      /* Separate checking if on the curve and if done with curve
           because we could be done before ever incrementing u_0 */
      // Check if done with current curve
      if( i == pop.calcBestIndex() && (curve.u_0 > curve.u_target || estimateIfOnCurve(ms, curve) == 3) )
      {
        ////ROS_INFO("Done with curve, u_0: %f", curve.u_0);
        curve = handleCurveEnd(pop.get(i));
      } // end if done with 1st curve
      else
      {
        ////ROS_INFO("Not done with curve");
      }

      ////ROS_INFO("Curve after adapting: %s", utility_.toString(curve).c_str());
      result.push_back(curve);
    } // end if trajectory has curve

    // Else if there is no curve, push on a blank one
    else 
    {
      ////ROS_INFO("No curve");
      result.push_back(blank);
    } // end else no curve
  } // end for

  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::adaptCurves");
  }

  return result;
} // End adaptCurves



void Planner::adaptPopulationOOP(const MotionState& ms, const ros::Duration& d)
{
  if(print_enter_exit_)
  {
    ROS_INFO("In adaptPopulation");
  }

  //std::vector<Path> paths = adaptPaths(population_, ms, d);
  std::vector<Path> paths;
  adaptPathsOOP(ms, d, paths);
  std::vector<ramp_msgs::BezierCurve> curves;
  if(population_.type_ != HOLONOMIC)
  {
    adaptCurvesOOP(ms, d, curves);
  }
  
  // Create the vector to hold updated trajectories
  std::vector<ramp_msgs::TrajectoryRequest> tr_reqs;
  
  //ROS_INFO("paths.size(): %i curves.size(): %i", (int)paths.size(), (int)curves.size());
  // For each path, get a trajectory request
  for(uint16_t i=0;i<population_.size();i++) 
  {
    //ROS_INFO("In 1st for, i: %i", (int)i);
      
    // Add on the curve if necessary
    std::vector<ramp_msgs::BezierCurve> c;
    if(population_.type_ != HOLONOMIC)
    {
      c.push_back(curves.at(i));
    }

    ramp_msgs::TrajectoryRequest tr;
    buildTrajectoryRequestOOP(paths.at(i), c, tr);
    tr.segments = 2;

    tr_reqs.push_back(tr);
  }
  
  // Get the new trajectories
  std::vector<RampTrajectory> updatedTrajecs;
  requestTrajectoryOOP(tr_reqs, updatedTrajecs);

  for(uint16_t i=0;i<updatedTrajecs.size();i++)
  {
    //ROS_INFO("In 2nd for, i: %i", (int)i);
      // Set temporary evaluation results - need to actually call requestEvaluation to get actual fitness
    updatedTrajecs.at(i).msg_.fitness   = population_.get(i).msg_.fitness;
    updatedTrajecs.at(i).msg_.feasible  = population_.get(i).msg_.feasible;
    updatedTrajecs.at(i).msg_.t_start   = ros::Duration(t_fixed_cc_);
  } // end for
  
  //////ROS_INFO("updatedTrajecs size: %i", (int)updatedTrajecs.size());
  // Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);
  
  //////ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  
  if(print_enter_exit_)
  {
    ROS_INFO("Exiting adaptPopulationOOP");
  }
}

/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
// Not const because it calls requestTrajectory which can call getIRT
const Population Planner::adaptPopulation(const Population pop, const MotionState ms, const ros::Duration d) 
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In adaptPopulation");
  }

  //////ROS_INFO("pop: %s", pop.toString().c_str());
  //////ROS_INFO("startPlanning_: %s \nduration: %f", startPlanning_.toString().c_str(), d.toSec());
  Population result = pop;
  
  //////ROS_INFO("Before adaptPaths, paths.size(): %i", (int)pop.paths_.size());
  //////ROS_INFO("Before adaptPaths, paths.size(): %i", (int)result.paths_.size());

  // Find how long we've been moving on the curve - how long between CCs minus the start of the curve
  //ros::Duration curveD = ros::Duration(d.toSec() - population_.getBest().msg_.curves.at(0).controlPoints.at(0).time);
  //////ROS_INFO("curveD: %f", curveD.toSec());
 
  // Adapt the paths and curves
  std::vector<Path> paths                       = adaptPaths  (pop, ms, d);
  std::vector<ramp_msgs::BezierCurve> curves;
  if(population_.type_ != HOLONOMIC)
  {
    curves                                      = adaptCurves (pop, ms, d);
  }

  result.paths_ = paths;

  // Create the vector to hold updated trajectories
  std::vector<ramp_msgs::TrajectoryRequest> tr_reqs;

  /*////ROS_INFO("pop.calcBestIndex(): %i", pop.calcBestIndex());
  ////ROS_INFO("paths.size(): %i", (int)paths.size());
  ////ROS_INFO("curves.size(): %i", (int)curves.size());*/
  // For each path, get a trajectory
  for(uint16_t i=0;i<pop.size();i++) 
  {
    RampTrajectory temp, tempTraj = pop.get(i);
    //////ROS_INFO("Getting trajectory %i", (int)i);
      
    // Add on the curve if necessary
    std::vector<ramp_msgs::BezierCurve> c;
    if(population_.type_ != HOLONOMIC)
    {
      c.push_back(curves.at(i));
    }

    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(paths.at(i), c);
    tr.segments = 2;

    tr_reqs.push_back(tr);
  }

  
  std::vector<RampTrajectory> updatedTrajecs = requestTrajectory(tr_reqs);
  for(uint16_t i=0;i<updatedTrajecs.size();i++)
  {
    ////ROS_INFO("In for, i: %i", i);
      // Set temporary evaluation results - need to actually call requestEvaluation to get actual fitness
    updatedTrajecs.at(i).msg_.fitness = result.get(i).msg_.fitness;
    updatedTrajecs.at(i).msg_.feasible = result.get(i).msg_.feasible;
    updatedTrajecs.at(i).msg_.t_start = ros::Duration(t_fixed_cc_);
  } // end for

  //////ROS_INFO("updatedTrajecs size: %i", (int)updatedTrajecs.size());
  // Replace the population's trajectories_ with the updated trajectories
  result.replaceAll(updatedTrajecs);
  
  //////ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  
  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting adaptPopulation");
  }

  return result;
} // End adaptPopulation




const ramp_msgs::TrajectorySrv Planner::buildTrajectorySrv(const Path path, const int id) const 
{
  std::vector<ramp_msgs::BezierCurve> curves;
  return buildTrajectorySrv(path, curves, id);
}



const ramp_msgs::TrajectorySrv Planner::buildTrajectorySrv(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, const int id) const 
{
  ramp_msgs::TrajectorySrv result;
  result.request.reqs.push_back(buildTrajectoryRequest(path, curves, id));
  return result;
}



// TODO: Clean up
/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, const int id) const 
{
  ////ROS_INFO("In Planner::buildTrajectoryRequest");
  ramp_msgs::TrajectoryRequest result;

  result.path           = path.buildPathMsg();
  result.type           = population_.type_;

  // If path size > 2, assign a curve
  if(path.size() > 2) 
  {
    ////ROS_INFO("In if path.size() > 2)");

    // If it's the first time getting a curve 
    if(curves.size() == 0 || curves.at(0).segmentPoints.size() == 0) 
    {
      if(path.size() > 2) 
      {
        ////ROS_INFO("In temp curve");
        ramp_msgs::BezierCurve temp;
        
        temp.segmentPoints.push_back( path.msg_.points.at(0).motionState );
        temp.segmentPoints.push_back( path.msg_.points.at(1).motionState );
        temp.segmentPoints.push_back( path.msg_.points.at(2).motionState );
        
        result.bezierCurves.push_back(temp);
      }
    }
    else 
    {
      ////ROS_INFO("In else if path.size < 3");
      result.bezierCurves = curves;
    } // end else
  } // end if


  ////ROS_INFO("result.path: %s", utility_.toString(result.path).c_str());
  ////ROS_INFO("Exiting Planner::buildTrajectoryRequest");
  return result;
} // End buildTrajectoryRequest


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const int id) const 
{
  std::vector<ramp_msgs::BezierCurve> curves;
  return buildTrajectoryRequest(path, curves, id);
}



// Build a srv for 1 trajectory with 1-2 curves
void Planner::buildTrajectorySrvOOP(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, ramp_msgs::TrajectorySrv& result, const int id) const
{
  result.request.reqs.push_back(buildTrajectoryRequest(path, curves, id));
}

// Build a srv for 1 trajectory with no curves
void Planner::buildTrajectorySrvOOP(const Path path, ramp_msgs::TrajectorySrv& result, const int id) const
{
  std::vector<ramp_msgs::BezierCurve> curves;
  buildTrajectorySrvOOP(path, curves, result, id);
}


// Build a request for 1 trajectory with 1-2 curves
void Planner::buildTrajectoryRequestOOP(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, ramp_msgs::TrajectoryRequest& result, const int id) const
{
  result.path           = path.buildPathMsg();
  result.type           = population_.type_;

  // If path size > 2, assign a curve
  if(path.size() > 2) 
  {
    ////ROS_INFO("In if path.size() > 2)");

    // If it's the first time getting a curve 
    if(curves.size() == 0 || curves.at(0).segmentPoints.size() == 0) 
    {
      if(path.size() > 2) 
      {
        ////ROS_INFO("In temp curve");
        ramp_msgs::BezierCurve temp;
        
        temp.segmentPoints.push_back( path.msg_.points.at(0).motionState );
        temp.segmentPoints.push_back( path.msg_.points.at(1).motionState );
        temp.segmentPoints.push_back( path.msg_.points.at(2).motionState );
        
        result.bezierCurves.push_back(temp);
      }
    }
    else 
    {
      ////ROS_INFO("In else if path.size < 3");
      result.bezierCurves = curves;
    } // end else
  } // end if


  ////ROS_INFO("result.path: %s", utility_.toString(result.path).c_str());
  ////ROS_INFO("Exiting Planner::buildTrajectoryRequestOOP");
}

// Build a request for 1 trajectory with 0 curves
void Planner::buildTrajectoryRequestOOP(const Path path, ramp_msgs::TrajectoryRequest& result, const int id) const
{
  std::vector<ramp_msgs::BezierCurve> curves;
  buildTrajectoryRequestOOP(path, curves, result, id);
}


const ramp_msgs::EvaluationSrv Planner::buildEvaluationSrv(const RampTrajectory trajec)
{
  std::vector<RampTrajectory> t;
  t.push_back(trajec);
  return buildEvaluationSrv(t);
}

const ramp_msgs::EvaluationSrv Planner::buildEvaluationSrv(const std::vector<RampTrajectory> trajecs)
{
  ramp_msgs::EvaluationSrv result;
  std::vector<ramp_msgs::EvaluationRequest> reqs;
  for(uint8_t i=0;i<trajecs.size();i++)
  {
    reqs.push_back(buildEvaluationRequest(trajecs.at(i)));
  }

  result.request.reqs = reqs;
  return result;
}


/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) const
{
  //ROS_INFO("In Planner::buildEvaluationRequest");
  ramp_msgs::EvaluationRequest result;

  result.trajectory   = trajec.msg_;
  result.currentTheta = latestUpdate_.msg_.positions.at(2);
  if(movingOn_.msg_.trajectory.points.size() > 0)
  {
    result.theta_cc     = 
      movingOn_.msg_.trajectory.points.at(movingOn_.msg_.trajectory.points.size()-1).positions.at(2);
  }
  else
  {
    result.theta_cc = result.currentTheta;
  }

  for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    result.obstacle_trjs.push_back(ob_trajectory_.at(i).msg_);
  }
  
  //ROS_INFO("imminent_collision: %s", imminent_collision_ ? "True" : "False");

  result.imminent_collision = imminent_collision_;
  result.coll_dist = COLL_DISTS[i_COLL_DISTS_];

  //ROS_INFO("Exiting Planner::buildEvaluationRequest");
  return result;
} // End buildEvaluationRequest

void Planner::buildEvaluationSrvOOP(std::vector<RampTrajectory>& trajecs, ramp_msgs::EvaluationSrv& srv) const
{
  for(uint16_t i=0;i<trajecs.size();i++)
  {
    ramp_msgs::EvaluationRequest req;
    buildEvaluationRequestOOP(trajecs[i], req);
    srv.request.reqs.push_back(req);
    //srv.request.reqs.push_back(buildEvaluationRequest(trajecs[i]));
  }
}

void Planner::buildEvaluationSrvOOP(const RampTrajectory& trajec, ramp_msgs::EvaluationSrv& result) const
{
  std::vector<RampTrajectory> t;
  t.push_back(trajec);
  buildEvaluationSrvOOP(t, result);
}

void Planner::buildEvaluationRequestOOP(const RampTrajectory& trajec, ramp_msgs::EvaluationRequest& result, bool full) const
{
  //ROS_INFO("In Planner::buildEvaluationRequestOOP(const RampTrajectory&, EvaluationRequest&, bool)");
  //ROS_INFO("trajec: %s", trajec.toString().c_str());
  //ROS_INFO("full: %s", full ? "True" : "False");

  result.trajectory   = trajec.msg_;
  result.currentTheta = latestUpdate_.msg_.positions[2]; 

  if(movingOn_.msg_.trajectory.points.size() > 0)
  {
    result.theta_cc = 
      movingOn_.msg_.trajectory.points[movingOn_.msg_.trajectory.points.size()-1].positions[2];
  }
  else
  {
    result.theta_cc = result.currentTheta;
  }

  for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    result.obstacle_trjs.push_back(ob_trajectory_[i].msg_);
  }

  ////ROS_INFO("imminent_collision: %s", imminent_collision_ ? "True" : "False");
  
  result.imminent_collision = imminent_collision_;
  result.coll_dist = COLL_DISTS[i_COLL_DISTS_];

  // full_eval is for predicting segments that are not generated
  result.full_eval = full;

  // consider_trans for trajs including switches
  result.consider_trans = true;
  result.trans_possible = trajec.transitionTraj_.trajectory.points.size() > 0;

  //ROS_INFO("transitionTraj: %s", utility_.toString(trajec.transitionTraj_).c_str());
  //ROS_INFO("Exiting Planner::buildEvaluationRequestOOP(const RampTrajectory&, EvaluationRequest&, bool)");
}




/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
// Not const because it calls getIRT() to get an index for the trajectory if an id is not passed in
const std::vector<RampTrajectory> Planner::requestTrajectory(ramp_msgs::TrajectorySrv& tr, const int id) 
{
  ////ROS_INFO("In Planner::requestTrajectory(ramp_msgs::TrajectorySrv)");
  std::vector<RampTrajectory> result;
  //std::cout<<"\nid: "<<id;
  
  ros::Time t_start = ros::Time::now();
  if(h_traj_req_->request(tr)) 
  {
    trajec_durs_.push_back(ros::Time::now() - t_start);
    
    ////ROS_INFO("tr.request.reqs.size(): %i", (int)tr.request.reqs.size());
    ////ROS_INFO("tr.resps.size(): %i", (int)tr.response.resps.size());
    for(uint8_t i=0;i<tr.response.resps.size();i++)
    {
      RampTrajectory temp;

      // Set the actual trajectory msg
      temp.msg_         = tr.response.resps.at(i).trajectory;

      // Set things the traj_gen does not have
      temp.msg_.t_start = ros::Duration(t_fixed_cc_);

      // Set the paths (straight-line and bezier)
      temp.msg_.holonomic_path = tr.request.reqs.at(i).path;

      // Set the ID of the trajectory
      if(id != -1) 
      {
        temp.msg_.id = id;
      }
      else 
      {
        temp.msg_.id = getIRT();
      }

      result.push_back(temp);
    } // end for
  } // end if
  else 
  {
    //ROS_ERROR("An error occurred when requesting a trajectory");
  }

  ////ROS_INFO("Exiting Planner::requestTrajectory, t_start: %f", result.msg_.t_start.toSec());
  return result;
}

void Planner::requestTrajectoryOOP(ramp_msgs::TrajectorySrv& tr, std::vector<RampTrajectory>& result, const int id)
{
  ////ROS_INFO("In Planner::requestTrajectory(ramp_msgs::TrajectorySrv)");
  //std::cout<<"\nid: "<<id;
  
  ros::Time t_start = ros::Time::now();
  if(h_traj_req_->request(tr)) 
  {
    trajec_durs_.push_back(ros::Time::now() - t_start);
    
    ////ROS_INFO("tr.request.reqs.size(): %i", (int)tr.request.reqs.size());
    ////ROS_INFO("tr.resps.size(): %i", (int)tr.response.resps.size());
    for(uint8_t i=0;i<tr.response.resps.size();i++)
    {
      RampTrajectory temp;

      // Set the actual trajectory msg
      temp.msg_             = tr.response.resps.at(i).trajectory;

      // Set things the traj_gen does not have
      temp.msg_.t_start     = ros::Duration(t_fixed_cc_);

      // Set the paths (straight-line and bezier)
      temp.msg_.holonomic_path  = tr.request.reqs.at(i).path;

      // Set the ID of the trajectory
      if(id != -1) 
      {
        temp.msg_.id = id;
      }
      else 
      {
        temp.msg_.id = getIRT();
      }

      result.push_back(temp);
    } // end for
  } // end if
  else 
  {
    //ROS_ERROR("An error occurred when requesting a trajectory");
  }

  ////ROS_INFO("Exiting Planner::requestTrajectory, t_start: %f", result.msg_.t_start.toSec());
}





const std::vector<RampTrajectory> Planner::requestTrajectory(std::vector<ramp_msgs::TrajectoryRequest> trs)
{
  ////ROS_INFO("In Planner::requestTrajectory(vector<ramp_msgs::TrajectoryRequest>)");
  std::vector<RampTrajectory> result;
  ramp_msgs::TrajectorySrv srv;

  for(uint8_t i=0;i<trs.size();i++)
  {
    srv.request.reqs.push_back(trs.at(i));
  }

  result = requestTrajectory(srv);

  return result;
}



void Planner::requestTrajectoryOOP(std::vector<ramp_msgs::TrajectoryRequest>& trs, std::vector<RampTrajectory>& result)
{
  ////ROS_INFO("In Planner::requestTrajectory(vector<ramp_msgs::TrajectoryRequest>)");
  ramp_msgs::TrajectorySrv srv;

  for(uint8_t i=0;i<trs.size();i++)
  {
    srv.request.reqs.push_back(trs.at(i));
  }

  requestTrajectoryOOP(srv, result);
}




const RampTrajectory Planner::requestTrajectory(ramp_msgs::TrajectoryRequest tr)
{
  ramp_msgs::TrajectorySrv srv;
  srv.request.reqs.push_back(tr);

  std::vector<RampTrajectory> vec = requestTrajectory(srv);
  return vec.at(0);
}



void Planner::requestTrajectoryOOP(ramp_msgs::TrajectoryRequest& tr, RampTrajectory& result)
{
  ramp_msgs::TrajectorySrv srv;
  srv.request.reqs.push_back(tr);

  std::vector<RampTrajectory> vec;
  requestTrajectoryOOP(srv, vec);
  
  result = vec.at(0);
}



const RampTrajectory Planner::requestTrajectory(const Path p, const int id) 
{
  ////ROS_INFO("In Planner::requestTrajectory(Path p, int id");
  ////ROS_INFO("Path p: %s", p.toString().c_str());
  ramp_msgs::TrajectorySrv tr = buildTrajectorySrv(p);
  std::vector<RampTrajectory> vec = requestTrajectory(tr, id);
  return vec.at(0);
}


void Planner::requestTrajectoryOOP(const Path p, RampTrajectory& result, const int id)
{
  ramp_msgs::TrajectorySrv tr;
  buildTrajectorySrvOOP(p, tr);

  std::vector<RampTrajectory> vec;
  requestTrajectoryOOP(tr, vec, id);
  
  result = vec.at(0);
}





const std::vector<RampTrajectory> Planner::requestEvaluation(std::vector<RampTrajectory> trajecs)
{
  std::vector<RampTrajectory> result;

  ramp_msgs::EvaluationSrv srv = buildEvaluationSrv(trajecs);

  if(h_eval_req_->request(srv))
  {
    for(uint8_t i=0;i<trajecs.size();i++)
    {
      RampTrajectory rt = srv.request.reqs.at(i).trajectory;
      rt.msg_.holonomic_path = trajecs.at(i).msg_.holonomic_path;
      rt.msg_.fitness = srv.response.resps.at(i).fitness;
      rt.msg_.feasible = srv.response.resps.at(i).feasible;
      rt.msg_.t_firstCollision = srv.response.resps.at(i).t_firstCollision;
      ////ROS_INFO("t_firstCollision: %f", srv.response.resps.at(i).t_firstCollision.toSec());
      result.push_back(rt);
    }
  } 
  else 
  {
    //ROS_ERROR("An error occurred when evaluating a trajectory");
  }

  return result;
}

/** Request an evaluation */
const RampTrajectory Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) 
{
  ////ROS_INFO("In Planner::requestEvaluation");
  RampTrajectory result = er.trajectory; 
  ////ROS_INFO("result.t_start: %f", result.msg_.t_start.toSec());
  
  
  ramp_msgs::EvaluationSrv srv;
  srv.request.reqs.push_back(er);

  if(h_eval_req_->request(srv)) 
  {
    result.msg_.fitness           = srv.response.resps.at(0).fitness;
    result.msg_.feasible          = srv.response.resps.at(0).feasible;
    result.msg_.t_firstCollision  = srv.response.resps.at(0).t_firstCollision;
    ////ROS_INFO("t_firstCollision: %f", srv.response.resps.at(0).t_firstCollision.toSec());
  }
  else 
  {
    //ROS_ERROR("An error occurred when evaluating a trajectory");
  }
  
  ////ROS_INFO("Exiting Planner::requestEvaluation");
  return result;
}


const RampTrajectory Planner::requestEvaluation(const RampTrajectory traj) 
{
  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(traj);
  ros::Time start = ros::Time::now();
  RampTrajectory result           = requestEvaluation(er);
  eval_durs_.push_back( ros::Time::now() - start );

  // Set non-evaluation related members
  result.msg_.holonomic_path      = traj.msg_.holonomic_path;
  result.msg_.i_subPopulation = traj.msg_.i_subPopulation; 

  return result;
}


/** This method initializes the T_w_odom_ transform object */
void Planner::setT_base_w(std::vector<double> base_pos) {
  T_w_odom_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_w_odom_.setOrigin(  tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
const unsigned int Planner::getIRT() { return i_rt++; }


void Planner::obICCallback(const ros::TimerEvent& e)
{
  //ROS_INFO("Time since last obICCallback: %f", (ros::Time::now() - t_prevObIC_).toSec());
  t_prevObIC_ = ros::Time::now();
  ////ROS_INFO("In Planner::obICCallback");
  double dist_theshold = 0.6f;
  std_msgs::Bool ob_ic;

  double min_dist;
  if(ob_trajectory_.size() > 0)
    min_dist = utility_.positionDistance(ob_trajectory_.at(0).msg_.trajectory.points.at(0).positions, latestUpdate_.msg_.positions);
 
  for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    double dist = utility_.positionDistance(ob_trajectory_.at(i).msg_.trajectory.points.at(0).positions, latestUpdate_.msg_.positions);
    //ROS_INFO("ob %i dist: %f", (int)i, dist);
    if(i < ob_dists_.size())
    {
      ob_dists_.at(i) = dist;
    }
    else
    {
      ob_dists_.push_back(dist);
    }
    if(fabs(ob_dists_.at(i)) < dist_theshold)
    {
      ////ROS_INFO("Ob IC: True");
      ob_ic.data = true;
    }
    else
    {
      ////ROS_INFO("Ob IC: False");
      ob_ic.data = false;
    }

    if(dist < min_dist)
    {
      min_dist = dist;
    }
    h_control_->sendObIC(i, ob_ic);
  }

  if(min_dist > COLL_DISTS[0])
  {
    i_COLL_DISTS_ = 0; 
  }
  else
  {
    i_COLL_DISTS_ = 1;
  }
  
  ////ROS_INFO("Exiting Planner::obICCallback");
}



void Planner::resetStart() 
{

  ROS_INFO("In Planner::resetStart");
  ROS_INFO("Pop: %s", population_.toString().c_str());
  startPlanning_ = latestUpdate_;
  adaptPopulationOOP(startPlanning_, ros::Duration(0.0001));
  ROS_INFO("After reset, Pop: %s", population_.toString().c_str());
  ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  reset_ = true;

}


/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) 
{
  ros::Duration d = ros::Time::now() - t_prevIC_;
  t_prevIC_ = ros::Time::now();
  ROS_INFO("In imminentCollisionCallback");
  ROS_INFO("Time since last: %f", d.toSec());

  std_msgs::Bool ic;

  double time_threshold = controlCycle_.toSec();
    
  /*//ROS_WARN("Robot trajectory: %s", movingOn_.toString().c_str());
  for(int o=0;o<ob_trajectory_.size();o++)
  {
    //ROS_WARN("Obstacle trajectory %i: %s", o, ob_trajectory_.at(o).toString().c_str());
  }*/

  if(ob_trajectory_.size() > 0 && moving_on_coll_ && (movingOn_.msg_.t_firstCollision.toSec() < time_threshold
    || (movingOn_.msg_.t_firstCollision.toSec() - (ros::Time::now().toSec()-t_prevCC_.toSec())) < time_threshold))
  {
    ROS_WARN("IC: moving_on_coll_: %s t_firstCollision: %f Elapsed time: %f", moving_on_coll_ ? "True" : "False", movingOn_.msg_.t_firstCollision.toSec(), (ros::Time::now().toSec()-t_prevCC_.toSec()));

    ic.data = true;
    imminent_collision_ = true;

    if(!reset_ && sqrt( pow( latestUpdate_.msg_.velocities[0], 2) + pow( latestUpdate_.msg_.velocities[1], 2) ) < 0.01 && fabs( latestUpdate_.msg_.velocities[2] ) < 0.01)
    {
      resetStart();
    }
    num_ops_ = 6;
  }

  // Do not set any imminent collision values
  else 
  {
    ROS_INFO("No imminent collision, t_firstCollision: %f", movingOn_.msg_.t_firstCollision.toSec());
    //ic.data = false;
    imminent_collision_ = false;
    //num_ops_ = 5;
  }

  h_control_->sendIC(ic);

  //ROS_INFO("Exiting Planner::imminentCollisionCallback");
}




/** 
 * Sets the latest update member
 * and transformes it by T_base_w because 
 * updates are relative to odometry frame
 * */
void Planner::updateCallback(const ramp_msgs::MotionState& msg) {
  t_prev_update_ = ros::Time::now();
  //ROS_INFO("In Planner::updateCallback");
  //ROS_INFO("Time since last: %f", (ros::Time::now()-t_prev_update_).toSec());

 
  if(msg.positions.size() < 3 ||
     msg.velocities.size() < 3 ||
     msg.accelerations.size() < 3 )
  { 
    //ROS_ERROR("Odometry message from ramp_control does not have all DOFs: %s", utility_.toString(msg).c_str());
  }
  else 
  {
    latestUpdate_ = msg;

    // Transform configuration from odometry to world coordinates
    latestUpdate_.transformBase(T_w_odom_);

    // Set proper velocity values
    latestUpdate_.msg_.velocities.at(0) = msg.velocities.at(0) * 
                                          cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.velocities.at(1) = msg.velocities.at(0) * 
                                          sin(latestUpdate_.msg_.positions.at(2));

    // Set proper acceleration values
    latestUpdate_.msg_.accelerations.at(0) = msg.accelerations.at(0) * 
                                             cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.accelerations.at(1) = msg.accelerations.at(0) * 
                                             sin(latestUpdate_.msg_.positions.at(2));

    //ROS_INFO("New latestUpdate_: %s", latestUpdate_.toString().c_str());
  } // end else
  
  //ROS_INFO("Exiting Planner::updateCallback");
} // End updateCallback






/** This method sets random values for the position vector of ms */
const MotionState Planner::randomizeMSPositions(const MotionState ms) const {
  MotionState result = ms;
  result.msg_.positions.clear();

  for(unsigned int i=0;i<ranges_.size();i++) 
  {
    result.msg_.positions.push_back(ranges_.at(i).random());
  }

  return result;
} // End randomizeMotionState


/****************************************************
 ************** Initialization Methods **************
 ****************************************************/



void Planner::initStartGoal(const MotionState s, const MotionState g) {
  start_  = s;
  goal_   = g; 

  m_cc_ = start_;
  startPlanning_ = start_;

  latestUpdate_ = start_;
}


/** Initialize the handlers and allocate them on the heap */
void Planner::init(const uint8_t i, const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r, const int population_size, const bool sub_populations, const std::vector<tf::Transform> ob_T_odoms, const TrajectoryType pop_type, const int gens_before_cc, const double t_pc_rate, const double t_fixed_cc, const bool errorReduction) {
  //////ROS_INFO("In Planner::init");

  // Set ID
  id_ = i;

  // Initialize the handlers
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
  modifier_   = new Modifier(h, num_ops_);

  // Initialize the timers, but don't start them yet
  controlCycle_       = ros::Duration(t_fixed_cc);
  controlCycleTimer_  = h.createTimer(ros::Duration(controlCycle_), 
                                     &Planner::controlCycleCallback, this);
  controlCycleTimer_.stop();

  //planningCycle_      = ros::Duration(1.f/t_pc_rate);
  //planningCycleTimer_ = h.createTimer(ros::Duration(planningCycle_), &Planner::planningCycleCallback, this);
  //planningCycleTimer_.stop();

  imminentCollisionTimer_ = h.createTimer(ros::Duration(imminentCollisionCycle_), 
                                          &Planner::imminentCollisionCallback, this);
  imminentCollisionTimer_.stop();

  ob_dists_timer_ = h.createTimer(ros::Duration(ob_dists_timer_dur_), &Planner::obICCallback, this);
  ob_dists_timer_.stop();



  // Set the ranges vector
  ranges_ = r;

  // Initialize the start and goal
  initStartGoal(s, g);

  // Set the base transformation
  setT_base_w(start_.msg_.positions);

  //setOb_T_w_odom();

  // Set misc members
  populationSize_       = population_size;
  subPopulations_       = sub_populations;
  ob_T_w_odom_          = ob_T_odoms;
  pop_type_             = pop_type;
  generationsBeforeCC_  = gens_before_cc;
  t_fixed_cc_           = t_fixed_cc;
  errorReduction_       = errorReduction;
  generationsPerCC_     = controlCycle_.toSec() / planningCycle_.toSec();
} // End init






/** Place code to seed population here */
void Planner::seedPopulation() {

  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(2.);
  kp.motionState.positions.push_back(2.);
  kp.motionState.positions.push_back(0.785); // 26 degrees 
  
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  
  ramp_msgs::KnotPoint kp1;
  
  kp1.motionState.positions.push_back(3.);
  kp1.motionState.positions.push_back(0.);
  kp1.motionState.positions.push_back(PI/4);
  
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all;
  all.push_back(start_);
  all.push_back(kp);
  //all.push_back(kp1);
  all.push_back(goal_);

  Path p1(all);


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(0.);
  kp2.motionState.positions.push_back(3.);
  kp2.motionState.positions.push_back(PI/2.);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(start_);
  all2.push_back(kp2);
  all2.push_back(goal_);

  Path p2(all2);

  ramp_msgs::KnotPoint kp3;
  
  kp3.motionState.positions.push_back(2.f);
  kp3.motionState.positions.push_back(0.f);
  kp3.motionState.positions.push_back(0.f);
  
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  std::vector<KnotPoint> all3;
  all3.push_back(start_);
  all3.push_back(kp3);
  all3.push_back(goal_);

  Path p3(all3);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p2);
  paths.push_back(p3);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  population_.clear();
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    trajec = requestEvaluation(trajec);
    ////ROS_INFO("Seede trajec: %s", trajec.toString().c_str());
    population_.add(trajec); 
  
  } // end for
  /************************************/

} // End seedPopulation



/** Will seed population with a straight-line trajectory to the goal */
void Planner::seedPopulationTwo() {


  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(0.707); // 80 degrees 
  

  std::vector<KnotPoint> all;
  all.push_back(startPlanning_);
  all.push_back(kp);
  all.push_back(goal_);

  Path p1(all);


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(1);
  kp2.motionState.positions.push_back(-2);
  kp2.motionState.positions.push_back(2.21431);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(startPlanning_);
  all2.push_back(kp2);
  all2.push_back(goal_);

  Path p2(all2);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p2);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    new_pop.push_back(evaluateTrajectory(trajec));
  
  } // end for
  /************************************/

  population_.replaceAll(new_pop);  
} // End seedPopulationTwo


const std::vector<MotionState> Planner::setMi(const RampTrajectory& trj_current) const 
{
  ////ROS_INFO("In Planner::setMi");
  ////ROS_INFO("trj_current: %s", trj_current.toString().c_str());
  std::vector<MotionState> result;
  
 
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) 
  {
    MotionState temp = movingOn_.getPointAtTime(planningCycle_.toSec()*(i+1));
    
    result.push_back(temp);

    ////ROS_INFO("m_i[%i]: %s", i, temp.toString().c_str());
  } // end for


  ////ROS_INFO("Exiting Planner::setMi");
  return result;
} // End setMi




/** Pass in entire RampTrajectory because we need the path info */
const ramp_msgs::BezierCurve Planner::replanCurve(const RampTrajectory trajec, const MotionState ms_start) const 
{
  ////ROS_INFO("In Planner::replanCurve");
  ramp_msgs::BezierCurve result = trajec.msg_.curves.at(0);

  // Get length of original curve's first segment
  double delta_x = trajec.msg_.holonomic_path.points.at(1).motionState.positions.at(0) - result.segmentPoints.at(0).positions.at(0);
  double delta_y = trajec.msg_.holonomic_path.points.at(1).motionState.positions.at(1) - result.segmentPoints.at(0).positions.at(1);
  double l = sqrt( pow(delta_x, 2) + pow(delta_y, 2) );

  double theta = ms_start.msg_.positions.at(2);

  double x = result.segmentPoints.at(0).positions.at(0) + l*cos(theta);
  double y = result.segmentPoints.at(0).positions.at(1) + l*sin(theta);

  ////ROS_INFO("delta_x: %f delta_y: %f l: %f theta: %f x: %f y: %f", delta_x, delta_y, l, theta, x, y);
  result.segmentPoints.at(1).positions.at(0) = x;
  result.segmentPoints.at(1).positions.at(1) = y;
  result.controlPoints.clear();
  result.ms_initialVA.velocities.clear();
  result.ms_initialVA.accelerations.clear();
  result.u_0 = 0;
  result.u_dot_0 = 0;
  result.u_values.clear();
  result.u_dot_max = 0.;
  result.u_target = 0.;
 



  return result;
}


/*const RampTrajectory Planner::replanTrajec(const RampTrajectory trajec, const MotionState ms_start) 
{
  ////ROS_INFO("In Planner::replanTrajec");

  RampTrajectory result = trajec;
  ////ROS_INFO("After setting result");
  ////ROS_INFO("Test my ID: pop best ID: %i", population_.getBest().msg_.id);
  ////ROS_INFO("result: %s", result.toString().c_str());
  ////ROS_INFO("result.curves.size: %i", (int)result.msg_.curves.size());
  
  ramp_msgs::KnotPoint kp_start;
  kp_start.motionState = ms_start.msg_;

  result.msg_.holonomic_path.start_ = kp_start;
  ////ROS_INFO("ms_start: %s", ms_start.toString().c_str());

  result.msg_.holonomic_path.points.erase(   result.msg_.holonomic_path.points.begin() );
  result.msg_.holonomic_path.points.insert(  result.msg_.holonomic_path.points.begin(), kp_start);

  double v = sqrt(  pow( ms_start.msg_.velocities.at(0), 2) + 
                    pow( ms_start.msg_.velocities.at(1), 2) );
  ////ROS_INFO("v: %f", v);

  // Replan the curve if it's the best trajectory
  if( trajec.equals(population_.getBest())  && 
      result.msg_.curves.size() > 0         && 
      v > 0.0001                            && 
      result.msg_.curves.at(0).u_0 < 0.001      ) 
  {
    result.msg_.curves.at(0) = replanCurve( trajec, ms_start );
    result.msg_.holonomic_path.msg_.points.erase(result.msg_.holonomic_path.msg_.points.begin()+1);

    KnotPoint m(result.msg_.curves.at(0).segmentPoints.at(1));
    result.msg_.holonomic_path.msg_.points.insert(result.msg_.holonomic_path.msg_.points.begin()+1, m.buildKnotPointMsg());
   
  }
  else 
  {
    ////ROS_INFO("Not replanning curve for trajec id: %i", trajec.msg_.id);
    ////ROS_INFO("v: %f curves.size(): %i", v, (int)result.msg_.curves.size());
    if(result.msg_.curves.size() > 0) 
    {
      ////ROS_INFO("curve.u_0: %f", result.msg_.curves.at(0).u_0);
    }
  }

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(result.msg_.holonomic_path, result.msg_.curves, trajec.msg_.id);

  //result = requestTrajectory(tr, result.msg_.id);
  result = requestTrajectory(tr);

  ////ROS_INFO("Replanned Trajec: %s", result.toString().c_str());

  ////ROS_INFO("Exiting Planner::replanTrajec");
  return result;
}*/




/*const std::vector<RampTrajectory> Planner::replanTrajecs(const std::vector<RampTrajectory> trajecs, const MotionState ms_start) {
  ////ROS_INFO("In Planner::replanTrajecs");
  std::vector<RampTrajectory> result;

  for(uint8_t i=0;i<trajecs.size();i++) {
    ////ROS_INFO("i: %i trajecs.size(): %i", (int)i, (int)trajecs.size());
    RampTrajectory temp = replanTrajec(trajecs.at(i), ms_start);
    result.push_back(temp);
  }

  ////ROS_INFO("Exiting Planner::replanTrajecs");
  return result;
}*/


/** This method will return a vector of trajectoies for the vector of paths */
const std::vector<RampTrajectory> Planner::getTrajectories(const std::vector<Path>& p) 
{
  //ROS_INFO("In Planner::getTrajectories");

  std::vector<RampTrajectory> result;

  ramp_msgs::TrajectorySrv tr_srv;

  // For each path
  for(unsigned int i=0;i<p.size();i++) 
  {
    ////ROS_INFO("i: %i p.size(): %i", (int)i, (int)p.size());
    // Get a trajectory
    //RampTrajectory temp = requestTrajectory(p.at(i));
    //result.push_back(temp);
 
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p.at(i));
    //ROS_INFO("Trajec Request path: %s", utility_.toString(tr.path).c_str());
    tr_srv.request.reqs.push_back(tr);
  } // end for
  //ROS_INFO("Outside of for-loop");

  //requestTrajectoryOOP(tr_srv, result);

  result = requestTrajectory(tr_srv);
  //ROS_INFO("Done with requestTrajectory");
  for(uint8_t i=0;i<result.size();i++)
  {
    //ROS_INFO("Path %i: %s", i, utility_.toString(result.at(i).msg_.holonomic_path).c_str());
  }

  //ROS_INFO("Exiting Planner::getTrajectories");
  return result;
} // End getTrajectories





/** This method will return a vector of trajectoies for the vector of paths */
// TODO: trajectoryrequest reference?
const std::vector<RampTrajectory> Planner::getTrajectories(std::vector<ramp_msgs::TrajectoryRequest>& tr) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<tr.size();i++) {
    
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(tr.at(i));
    result.push_back(temp);
  } // end for

  return result;
} // End getTrajectories




/** 
 * This function generates the initial population of trajectories,
 *  sets the paths in the Modifier class
 *  and evaluates the population
 **/
void Planner::initPopulation() 
{ 
  //ROS_INFO("In Planner::initPopulation");

  population_ = getPopulation(latestUpdate_, goal_, false);
  /*for(uint8_t i=0;i<population_.size();i++)
  {
    RampTrajectory temp = population_.get(i);
    temp.msg_.t_start = ros::Duration(0);
    population_.replace(i, temp);
  }*/

  //ROS_INFO("Pop paths_ size: %i", (int)population_.paths_.size());
  for(uint8_t i=0;i<population_.paths_.size();i++)
  {
    //ROS_INFO("Path %i: %s", i, population_.paths_.at(i).toString().c_str());
  }
  //ROS_INFO("Exiting Planner::initPopulation");
} // End init_population




const bool Planner::checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to) const {
  ////ROS_INFO("In Planner::CheckIfSwitchCurveNecessary");
  // TODO: lastestUpdate or from's first theta?
  double thetaToSwitch, thetaCurrent = latestUpdate_.msg_.positions.at(2);

  //////ROS_INFO("to.msg.trajectory.points.size(): %i", (int)to.msg_.trajectory.points.size());
  //////ROS_INFO("to.msg.i_knotPoints.size(): %i", (int)to.msg_.i_knotPoints.size());
  //if(to.msg_.i_knotPoints.size() > 1) {
    //////ROS_INFO("to.msg.i_knotPoints.at(1): %i", (int)to.msg_.i_knotPoints.at(1));
  //}

  int kp = 1; 
  // Check if 1st two positions in "to" trajectory are the same
  // If they are, the 2nd kp is the end of a rotation
  // Theta after rotating will tell us the theta needed to move on "to" trajectory
  if( to.msg_.i_knotPoints.size() > 1 && fabs(utility_.positionDistance( to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(0)).positions,
                                            to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(kp)).positions)) < 0.0001)
  {
    //////ROS_INFO("In if positions are the same");
    thetaToSwitch = to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1) ).positions.at(2);
  }
  else if( to.msg_.i_knotPoints.size() > 1) {
    //////ROS_INFO("In else positions are not the same");
    thetaToSwitch = utility_.findAngleFromAToB(
                          to.msg_.trajectory.points.at(0), 
                          to.msg_.trajectory.points.at(
                            to.msg_.i_knotPoints.at(kp)) ); 
  }
  else 
  {
    ////ROS_INFO("to trajec: %s", to.toString().c_str());
    thetaToSwitch = to.msg_.trajectory.points.at(0).positions.at(2);
  }


  ////ROS_INFO("thetaCurrent: %f thetaToSwitch: %f", thetaCurrent, thetaToSwitch);
  ////ROS_INFO("fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)): %f", fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)));


  // If a difference of 1 degree, compute a curve
  if(fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)) > 0.017) {
    //////ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning true");
    return true;
  }

  ////ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning false");
  return false;
}



void Planner::computeFullSwitchOOP(const RampTrajectory& from, const RampTrajectory& to, RampTrajectory& result)
{
  //ROS_INFO("In Planner::computeFullSwitchOOP");
  ////ROS_INFO("to: %s", to.toString().c_str());

  // Get transition trajectory
  ros::Time tt = ros::Time::now();
  std::vector<RampTrajectory> trajecs;
  switchTrajectoryOOP(from, to, trajecs);
  //////ROS_INFO("Time spent getting switch trajectory: %f", (ros::Time::now()-tt).toSec());

  // If a switch was possible
  if(trajecs.size() > 0)
  {
    RampTrajectory T_new = trajecs.at(1);
    Path p = T_new.msg_.holonomic_path;
    //////ROS_INFO("Before eval, T_new.path: %s", T_new.path_.toString().c_str());
    //ROS_INFO("to.msg.holonomic_path: %s", utility_.toString(to.msg_.holonomic_path).c_str());;

    // Evaluate T_new
    //ramp_msgs::EvaluationRequest er = buildEvaluationRequest(T_new);
    requestEvaluationOOP(T_new);

    // Set misc members
    T_new.transitionTraj_ = trajecs.at(0).msg_;
    T_new.msg_.holonomic_path = p.msg_;

    // Set result
    result                  = T_new;
    result.transitionTraj_  = trajecs.at(0).msg_;
    //////ROS_INFO("result.transitionTraj.size(): %i", (int)result.transitionTraj_.trajectory.points.size());

    //////ROS_INFO("After eval, T_new.path: %s", T_new.path_.toString().c_str());
  }

  // If a switch was not possible, just return
  // the holonomic trajectory
  else
  {
    //////ROS_WARN("A switch was not possible, returning \"to\" trajectory: %s", to.toString().c_str());
    result = to;
  }

  ////ROS_INFO("Full switch result: %s", result.toString().c_str());
  //ROS_INFO("Exiting Planner::computeFullSwitchOOP");
} // End computeFullSwitchOOP

/*
 * Difference between computeFullSwitch and switchTrajectory is the result contains the evaluation result
 */
const RampTrajectory Planner::computeFullSwitch(const RampTrajectory& from, const RampTrajectory& to) 
{
  ////ROS_INFO("In Planner::computeFullSwitch");
  RampTrajectory result;

  ////ROS_INFO("to: %s", to.toString().c_str());

  // Get transition trajectory
  ros::Time tt = ros::Time::now();
  std::vector<RampTrajectory> trajecs = switchTrajectory(from, to);
  //////ROS_INFO("Time spent getting switch trajectory: %f", (ros::Time::now()-tt).toSec());

  // If a switch was possible
  if(trajecs.size() > 0)
  {
    RampTrajectory T_new = trajecs.at(1);
    Path p = T_new.msg_.holonomic_path;
    //////ROS_INFO("Before eval, T_new.path: %s", T_new.path_.toString().c_str());

    // Evaluate T_new
    ramp_msgs::EvaluationRequest er = buildEvaluationRequest(T_new);
    T_new                           = requestEvaluation(er);

    // Set misc members
    T_new.transitionTraj_ = trajecs.at(0).msg_;
    T_new.msg_.holonomic_path = p.msg_;

    // Set result
    result                  = T_new;
    //////ROS_INFO("result.transitionTraj.size(): %i", (int)result.transitionTraj_.trajectory.points.size());

    //////ROS_INFO("After eval, T_new.path: %s", T_new.path_.toString().c_str());
  }

  // If a switch was not possible, just return
  // the holonomic trajectory
  else
  {
    //////ROS_WARN("A switch was not possible, returning \"to\" trajectory: %s", to.toString().c_str());
    return to;
  }


  ////ROS_INFO("Full switch result: %s", result.toString().c_str());
  ////ROS_INFO("Exiting Planner::computeFullSwitch");
  return result;
} // End computeFullSwitch



// TODO: Figure out issue with not needing a curve!
bool Planner::predictTransition(const RampTrajectory& from, const RampTrajectory& to, const double& t)
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::predictTransition, t: %f", t);
  }

  if(to.msg_.trajectory.points.size() == 0)
  {
    //ROS_WARN("to.msg_.trajectory.points.size() == 0");
    //ROS_WARN("Returning false");
    return false;
  }

  // Get the first two control points for the transition curve
  MotionState ms_startTrans = from.getPointAtTime(t);
  MotionState ms_endOfMovingOn = to.msg_.trajectory.points.size() > 0 ? 
    to.msg_.trajectory.points.at(0) : 
    from.msg_.trajectory.points.at(from.msg_.trajectory.points.size()-1);
  ////ROS_INFO("ms_startTrans: %s", ms_startTrans.toString().c_str());
  ////ROS_INFO("ms_endOfMovingOn: %s", ms_endOfMovingOn.toString().c_str());

 
  /*
   * If the robot does not have correct orientation to move on the first segment
   * then a curve cannot be planned.
   * return a blank trajectory
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    //ROS_WARN("Cannot plan a transition curve!");
    //ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    return false;
  }


  // Add points to segments
  std::vector<MotionState> segmentPoints;
  segmentPoints.push_back(ms_startTrans);
  segmentPoints.push_back(ms_endOfMovingOn);

  /*
   * Get 3rd control point
   * 2nd knot point should be the initial point on that trajectory's bezier 
   * Using start of Bezier rather than segment endpoint ensures that
   * the trajectory will end at the start of the Bezier
   */
  int i_goal = 1;
  if(to.msg_.i_knotPoints.size() == 1) 
  {
    i_goal = 0;
  }

  // Set third segment point
  // Can't really use holonomic path because 
  // 1) the 2nd segment end point will not match the start of the curve so traj. concatenation will always fail
  // 2) the 2nd segment will be much longer 
  MotionState g(to.msg_.trajectory.points.at(to.msg_.i_knotPoints.at(i_goal)));

  ////ROS_INFO("g: %s", g.toString().c_str());

  segmentPoints.push_back(g);

  /*
   * Check misc things like same orientation, duplicate knot points, speed too fast
   */

  // Check TODO
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    //ROS_WARN("Cannot plan a transition curve!");
    //ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    return false;
  }

  /*//ROS_INFO("Segment Points:");
  for(uint8_t i=0;i<segmentPoints.size();i++)
  {
    //ROS_INFO("Segment Point %i: %s", i, segmentPoints.at(i).toString().c_str());
  }*/

  /*
   * After getting both segments, check if they have the same orientation
   * If so, just return the rest of movingOn, no need to plan a transition trajectory
   */
  double thetaS1 = utility_.findAngleFromAToB(segmentPoints.at(0).msg_.positions, 
                                              segmentPoints.at(1).msg_.positions);
  double thetaS2 = utility_.findAngleFromAToB(segmentPoints.at(1).msg_.positions, 
                                              segmentPoints.at(2).msg_.positions);
  ////ROS_INFO("Theta 1: %f Theta 2: %f", thetaS1, thetaS2);
  if( fabs(utility_.findDistanceBetweenAngles(thetaS1, thetaS2)) < 0.13 )
  {
    ////ROS_WARN("Segments have the same orientation - no need to plan a transition curve, use a straight-line trajectory");
    return true;
  }


  // Check duplicates and speeds of segment points
  for(int i=0;i<segmentPoints.size()-1;i++)
  {
    MotionState a = segmentPoints.at(i);
    MotionState b = segmentPoints.at(i+1);

    // Check duplicate
    if(utility_.positionDistance(a.msg_.positions, b.msg_.positions) < 0.1)
    {
      //ROS_WARN("Will not plan a transition curve because there are duplicate segment points");
      //ROS_WARN("%s\n%s", a.toString().c_str(), b.toString().c_str());
      return false;
    }
  } // end for


  ////ROS_INFO("Done checking segments");
 


  /*
   * See if a curve can be planned
   */
  // 0.1 = lambda
  BezierCurve curve;
  for(float lambda=0.1;lambda < 0.85;lambda+=0.1f)
  {
    ////ROS_INFO("lambda: %f", lambda);
    curve.init(segmentPoints, lambda, ms_startTrans);
    if(curve.verify())
    {
      ////ROS_INFO("Curve formed for prediction: %s", utility_.toString(curve.getMsg()).c_str());
      ////ROS_INFO("Exiting Planner::predictTransition");
      return true;
    }
  }

  ////ROS_WARN("No possible curve could be planned");
  return false;
}


void Planner::switchTrajectoryOOP(const RampTrajectory& from, const RampTrajectory& to, std::vector<RampTrajectory>& result)
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::switchTrajectoryOOP");
  }
  
  ////ROS_INFO("from: %s\nto.path: %s", from.toString().c_str(), to.toString().c_str());

  /*
   * Find the best planning cycle to switch 
   */
  
  // TODO: Use current CC time or fixed cc time? Come back to this.
  //uint8_t deltasPerCC = (controlCycle_.toSec()+0.0001) / delta_t_switch_;
  uint8_t deltasPerCC = (t_fixed_cc_+0.0001) / delta_t_switch_;

  uint8_t delta_t_now = (ros::Time::now() - t_prevCC_).toSec() / delta_t_switch_;
  ////ROS_INFO("(ros::Time::now() - t_prevCC_): %f", (ros::Time::now() - t_prevCC_).toSec());
  
  double  delta_t     = ((deltasPerCC+1)*delta_t_switch_);
  ////ROS_INFO("deltasPerCC: %i delta_t_now: %i delta_t: %f", deltasPerCC, delta_t_now, delta_t);
  
  for(int i_delta_t=deltasPerCC-1; i_delta_t > (delta_t_now+1); i_delta_t--)
  {
    double t_pc = i_delta_t * delta_t_switch_;
    ////ROS_INFO("i_delta_t: %i delta_t_switch_: %f t_pc: %f", i_delta_t, delta_t_switch_, t_pc);

    MotionState ms = from.getPointAtTime(t_pc);

    if(predictTransition(from, to, t_pc))
    {
      ////ROS_INFO("Prediction successful - stopping at t_pc: %f", t_pc);
      delta_t = t_pc;
      break;
    }
    /*else
    {
      //ROS_INFO("Prediction returns false");
    }*/
  } // end for
 

  /*
   * Call getTransitionTrajectory
   * if we can find one before next CC
   */
  if(delta_t  < ((deltasPerCC+1)*delta_t_switch_))
  {
    //RampTrajectory switching  = getTransitionTrajectory(from, to, delta_t);
    //RampTrajectory full       = switching.clone();
    RampTrajectory switching, full;
    getTransitionTrajectoryOOP(from, to, delta_t, switching);
    full = switching;
    ////ROS_INFO("Switching trajectory: %s", switching.toString().c_str());

    // If robot is at goal, full should only be 1 point,
    // check for this to prevent crashing
    if(full.msg_.i_knotPoints.size() > 1)
    {
      // Keep a counter for the knot points
      // Start at 1 because that should be the starting knot point of the curve
      int c_kp = 1;
      
      ////ROS_INFO("c_kp: %i", c_kp);
      ////ROS_INFO("c_kp: %i i_knotPoints.size(): %i", c_kp, (int)to.msg_.i_knotPoints.size());
      
      ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());

      // Set full as the concatenating of switching and to
      full        = switching.concatenate(to, c_kp);
      
      ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());


      // Set the proper ID, path, and t_starts
      full.msg_.id          = to.msg_.id;
      full.msg_.holonomic_path  = to.msg_.holonomic_path;

      full.msg_.t_start       = ros::Duration(delta_t);
      switching.msg_.t_start  = full.msg_.t_start;

      if(full.transitionTraj_.curves.size() > 0)
      {
        full.transitionTraj_    = switching.msg_;
      }

      result.push_back(switching);
      result.push_back(full);
    } // end if size > 1
  } // end if switch possible
 

  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::switchTrajectoryOOP");
  }
} // End switchTrajectoryOOP

/*
 * Returns a vector of trajectories
 * Index 0 = transition trajectory
 * Index 1 = Full switching trajectory
 */
const std::vector<RampTrajectory> Planner::switchTrajectory(const RampTrajectory& from, const RampTrajectory& to) 
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::switchTrajectory");
  }
  
  ////ROS_INFO("from: %s\nto.path: %s", from.toString().c_str(), to.toString().c_str());
  std::vector<RampTrajectory> result;

  /*
   * Find the best planning cycle to switch 
   */
  
  // TODO: Use current CC time or fixed cc time? Come back to this.
  //uint8_t deltasPerCC = (controlCycle_.toSec()+0.0001) / delta_t_switch_;
  uint8_t deltasPerCC = (t_fixed_cc_+0.0001) / delta_t_switch_;

  uint8_t delta_t_now = (ros::Time::now() - t_prevCC_).toSec() / delta_t_switch_;
  ////ROS_INFO("(ros::Time::now() - t_prevCC_): %f", (ros::Time::now() - t_prevCC_).toSec());
  double  delta_t     = ((deltasPerCC+1)*delta_t_switch_);
  ////ROS_INFO("deltasPerCC: %i delta_t_now: %i delta_t: %f", deltasPerCC, delta_t_now, delta_t);
  
  for(int i_delta_t=deltasPerCC-1; i_delta_t > (delta_t_now+1); i_delta_t--)
  {
    double t_pc = i_delta_t * delta_t_switch_;
    ////ROS_INFO("i_delta_t: %i delta_t_switch_: %f t_pc: %f", i_delta_t, delta_t_switch_, t_pc);

    MotionState ms = from.getPointAtTime(t_pc);

    if(predictTransition(from, to, t_pc))
    {
      ////ROS_INFO("Prediction successful - stopping at t_pc: %f", t_pc);
      delta_t = t_pc;
      break;
    }
    /*else
    {
      //ROS_INFO("Prediction returns false");
    }*/
  } // end for
 

  /*
   * Call getTransitionTrajectory
   * if we can find one before next CC
   */
  if(delta_t  < ((deltasPerCC+1)*delta_t_switch_))
  {
    RampTrajectory switching  = getTransitionTrajectory(from, to, delta_t);
    RampTrajectory full       = switching.clone();
    ////ROS_INFO("Switching trajectory: %s", switching.toString().c_str());

    // If robot is at goal, full should only be 1 point,
    // check for this to prevent crashing
    if(full.msg_.i_knotPoints.size() > 1)
    {
      // Keep a counter for the knot points
      // Start at 1 because that should be the starting knot point of the curve
      int c_kp = 1;
      
      ////ROS_INFO("c_kp: %i", c_kp);
      ////ROS_INFO("c_kp: %i i_knotPoints.size(): %i", c_kp, (int)to.msg_.i_knotPoints.size());
      
      ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());

      // Set full as the concatenating of switching and to
      full        = switching.concatenate(to, c_kp);
      
      ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());


      // Set the proper ID, path, and t_starts
      full.msg_.id          = to.msg_.id;
      full.msg_.holonomic_path  = to.msg_.holonomic_path;

      full.msg_.t_start       = ros::Duration(delta_t);
      switching.msg_.t_start  = full.msg_.t_start;

      full.transitionTraj_    = switching.msg_;

      result.push_back(switching);
      result.push_back(full);
    } // end if size > 1
  } // end if switch possible
 

  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::switchTrajectory");
  }

  return result;
} // End switchTrajectory





void Planner::getTransitionTrajectoryOOP(const RampTrajectory& trj_movingOn, const RampTrajectory& trj_target, const double& t, RampTrajectory& result)
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::getTransitionTrajectoryOOP");
  }
  /*//ROS_INFO("t: %f", t);
  //ROS_INFO("trj_movingOn: %s", trj_movingOn.toString().c_str());
  //ROS_INFO("trj_target: %s", trj_target.toString().c_str());*/

  /* 
   * The segment points for a transition are
   * 1) Motion state at time t along trj_movingOn
   * 2) Motion state at the end of trj_movingOn
   * 3) Motion state of the target's first knot point (normally start of Bezier curve)
   * The robot must be oriented towards CP 2 at CP 1 in order for a curve to be possible
   * If the robot is not oriented this way, log a warning and return a blank trajectory
   */
 
  // Get the first two control points for the transition curve
  MotionState ms_startTrans = trj_movingOn.getPointAtTime(t);
  MotionState ms_endOfMovingOn = trj_target.msg_.trajectory.points.size() > 0 ? 
    trj_target.msg_.trajectory.points.at(0) : 
    trj_movingOn.msg_.trajectory.points.at(trj_movingOn.msg_.trajectory.points.size()-1);
  ////ROS_INFO("ms_startTrans: %s", ms_startTrans.toString().c_str());
  ////ROS_INFO("ms_endOfMovingOn: %s", ms_endOfMovingOn.toString().c_str());

 
  /*
   * If the robot does not have correct orientation to move on the first segment
   * then a curve cannot be planned.
   * return a blank trajectory
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    //////ROS_WARN("Cannot plan a transition curve!");
    //////ROS_WARN("Robot does not have correct orientation to move on first segment of a transition curve");
    //////ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    RampTrajectory blank;
    result = blank;
    return;
  }


  // Add points to segments
  std::vector<MotionState> segmentPoints;
  segmentPoints.push_back(ms_startTrans);
  segmentPoints.push_back(ms_endOfMovingOn);



  /*
   * Get 3rd control point
   * 2nd knot point should be the initial point on that trajectory's bezier 
   * Using start of Bezier rather than segment endpoint ensures that
   * the trajectory will end at the start of the Bezier
   */
  int i_goal = 1;
  if(trj_target.msg_.i_knotPoints.size() == 1) 
  {
    i_goal = 0;
  }


 
  // Get last segment point as start of Bezier curve
  // Can't really use holonomic path because 
  // 1) the 2nd segment end point will not match the start of the curve so traj. concatenation will always fail
  // 2) the 2nd segment will be much longer 
  MotionState g(trj_target.msg_.trajectory.points.at(trj_target.msg_.i_knotPoints.at(i_goal)));
  segmentPoints.push_back(g);

  /*//ROS_INFO("Segment points:");
  for(int i=0;i<segmentPoints.size();i++)
  {
    //ROS_INFO("Segment point [%i]: %s", i, segmentPoints.at(i).toString().c_str());
  }*/


  // Make the path of the transition curve
  Path p(segmentPoints);

  /*
   * After getting both segments, check if they have the same orientation
   * If so, just return the rest of movingOn, no need to plan a transition trajectory
   */
  double thetaS1 = utility_.findAngleFromAToB(segmentPoints.at(0).msg_.positions, 
                                              segmentPoints.at(1).msg_.positions);
  double thetaS2 = utility_.findAngleFromAToB(segmentPoints.at(1).msg_.positions, 
                                              segmentPoints.at(2).msg_.positions);
  ////ROS_INFO("Theta 1: %f Theta 2: %f", thetaS1, thetaS2);
  if( fabs(utility_.findDistanceBetweenAngles(thetaS1, thetaS2)) < 0.13 )
  {
    //ROS_WARN("Segments have the same orientation - no need to plan a transition curve, use a straight-line trajectory");
    //ROS_WARN("Removing the following point at index 1 of the Path: %s", p.at(1).toString().c_str());
    p.msg_.points.erase(p.msg_.points.begin()+1);
  }

  if(fabs(ms_endOfMovingOn.msg_.velocities[2]) > 0.1 && fabs(g.msg_.velocities[2]) > 0.1)
  {
    //ROS_WARN("Segment 2 is actually a curve, stopping transition trajectory at first segment");
    //p.msg_.points.erase(p.msg_.points.begin()+1);
    RampTrajectory blank;
    result = blank;
    return;
  }


  /*
   * Get curve
   */

  // Build request and get trajectory
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.type = TRANSITION;

  result = requestTrajectory(tr);

  ////ROS_INFO("trj_transition: %s", trj_transition.toString().c_str());
  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::getTransitionTrajectory");
  }
}

const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory trj_movingOn, 
    const RampTrajectory trj_target, const double t) 
{
  if(print_enter_exit_)
  {
    //ROS_INFO("In Planner::getTransitionTrajectory");
  }
  /*//ROS_INFO("t: %f", t);
  //ROS_INFO("trj_movingOn: %s", trj_movingOn.toString().c_str());
  //ROS_INFO("trj_target: %s", trj_target.toString().c_str());*/

  /* 
   * The segment points for a transition are
   * 1) Motion state at time t along trj_movingOn
   * 2) Motion state at the end of trj_movingOn
   * 3) Motion state of the target's first knot point (normally start of Bezier curve)
   * The robot must be oriented towards CP 2 at CP 1 in order for a curve to be possible
   * If the robot is not oriented this way, log a warning and return a blank trajectory
   */
 
  // Get the first two control points for the transition curve
  MotionState ms_startTrans = trj_movingOn.getPointAtTime(t);
  MotionState ms_endOfMovingOn = trj_target.msg_.trajectory.points.size() > 0 ? 
    trj_target.msg_.trajectory.points.at(0) : 
    trj_movingOn.msg_.trajectory.points.at(trj_movingOn.msg_.trajectory.points.size()-1);
  ////ROS_INFO("ms_startTrans: %s", ms_startTrans.toString().c_str());
  ////ROS_INFO("ms_endOfMovingOn: %s", ms_endOfMovingOn.toString().c_str());

 
  /*
   * If the robot does not have correct orientation to move on the first segment
   * then a curve cannot be planned.
   * return a blank trajectory
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    //////ROS_WARN("Cannot plan a transition curve!");
    //////ROS_WARN("Robot does not have correct orientation to move on first segment of a transition curve");
    //////ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    RampTrajectory blank;
    return blank;
  }


  // Add points to segments
  std::vector<MotionState> segmentPoints;
  segmentPoints.push_back(ms_startTrans);
  segmentPoints.push_back(ms_endOfMovingOn);



  /*
   * Get 3rd control point
   * 2nd knot point should be the initial point on that trajectory's bezier 
   * Using start of Bezier rather than segment endpoint ensures that
   * the trajectory will end at the start of the Bezier
   */
  int i_goal = 1;
  if(trj_target.msg_.i_knotPoints.size() == 1) 
  {
    i_goal = 0;
  }


 
  // Can't really use holonomic path because 
  // 1) the 2nd segment end point will not match the start of the curve so traj. concatenation will always fail
  // 2) the 2nd segment will be much longer 
  MotionState g(trj_target.msg_.trajectory.points.at(trj_target.msg_.i_knotPoints.at(i_goal)));
  //MotionState g(trj_target.msg_.holonomic_path.at(1).motionState_);
  segmentPoints.push_back(g);

  /*//ROS_INFO("Segment points:");
  for(int i=0;i<segmentPoints.size();i++)
  {
    //ROS_INFO("Segment point [%i]: %s", i, segmentPoints.at(i).toString().c_str());
  }*/


  // Make the path of the transition curve
  Path p(segmentPoints);

  /*
   * After getting both segments, check if they have the same orientation
   * If so, just return the rest of movingOn, no need to plan a transition trajectory
   */
  double thetaS1 = utility_.findAngleFromAToB(segmentPoints.at(0).msg_.positions, 
                                              segmentPoints.at(1).msg_.positions);
  double thetaS2 = utility_.findAngleFromAToB(segmentPoints.at(1).msg_.positions, 
                                              segmentPoints.at(2).msg_.positions);
  ////ROS_INFO("Theta 1: %f Theta 2: %f", thetaS1, thetaS2);
  if( fabs(utility_.findDistanceBetweenAngles(thetaS1, thetaS2)) < 0.13 )
  {
    //ROS_WARN("Segments have the same orientation - no need to plan a transition curve, use a straight-line trajectory");
    //ROS_WARN("Removing the following point at index 1 of the Path: %s", p.at(1).toString().c_str());
    p.msg_.points.erase(p.msg_.points.begin()+1);
  }

  if(fabs(ms_endOfMovingOn.msg_.velocities[2]) > 0.1 && fabs(g.msg_.velocities[2]) > 0.1)
  {
    //ROS_WARN("Segment 2 is actually a curve, stopping transition trajectory at first segment");
    //p.msg_.points.erase(p.msg_.points.begin()+1);
    RampTrajectory blank;
    return blank;
  }


  /*
   * Get curve
   */

  // Build request and get trajectory
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.type = TRANSITION;

  RampTrajectory trj_transition = requestTrajectory(tr);

  ////ROS_INFO("trj_transition: %s", trj_transition.toString().c_str());
  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::getTransitionTrajectory");
  }

  return trj_transition;
} // End getTransitionTrajectory




/******************************************************
 ****************** Modifying Methods *****************
 ******************************************************/


/** Modify a Path */
const std::vector<Path> Planner::modifyPath() 
{ 
  ////ROS_INFO("About to modify a path, pop is: %s\n%s", population_.get(0).toString().c_str(), population_.get(1).toString().c_str());
  return modifier_->perform(population_);
  //return modifier_->perform(population_at_cc_);
}



/** Modify a trajectory */ 
const std::vector<RampTrajectory> Planner::modifyTrajec() 
{
  ////ROS_INFO("In Planner::modifyTrajec");
  std::vector<RampTrajectory> result;

  // The process begins by modifying one or more paths
  ros::Time t_p = ros::Time::now();
  std::vector<Path> modded_paths = modifyPath();
  ////ROS_INFO("Number of modified paths: %i", (int)modded_paths.size());

  // For each targeted path,
  for(unsigned int i=0;i<modded_paths.size();i++) 
  {
    //std::cout<<"\nramp_planner: Modifying trajectory "<<(int)i;
    
    // Get trajectory
    //RampTrajectory temp = requestTrajectory(modded_paths.at(i));
    t_p = ros::Time::now();
    result.push_back(requestTrajectory(modded_paths[i]));
    ros::Time t_m = ros::Time::now();
    //ROS_INFO("t_m: %f", (t_m-t_p).toSec());
  } // end for
  
  ////ROS_INFO("Exiting Planner::modifyTrajec");
  return result;
} // End modifyTrajectory


void Planner::modifyTrajecOOP(std::vector<RampTrajectory>& result)
{
  ////ROS_INFO("In Planner::modifyTrajec");

  // The process begins by modifying one or more paths
  ros::Time t_p = ros::Time::now();
  std::vector<Path> modded_paths = modifyPath();
  ////ROS_INFO("Number of modified paths: %i", (int)modded_paths.size());


  ros::Time t_for = ros::Time::now();
  // For each targeted path,
  for(unsigned int i=0;i<modded_paths.size();i++) 
  {
    //std::cout<<"\nramp_planner: Modifying trajectory "<<(int)i;
    
    // Get trajectory
    //RampTrajectory temp = requestTrajectory(modded_paths.at(i));
    ros::Time t_for = ros::Time::now();
    result.push_back(requestTrajectory(modded_paths[i]));
    ros::Time t_a = ros::Time::now();
    //ROS_INFO("t_mod_oop: %f", (t_a-t_for).toSec());
  } // end for
  
  ////ROS_INFO("Exiting Planner::modifyTrajec");
}





/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  set the new best trajectory,
 *  and return the index of the new best trajectory */
const ModificationResult Planner::modification() 
{
  ros::Time t_m = ros::Time::now();
  ////ROS_INFO("In Planner::modification()");
  ModificationResult result;

  // Modify 1 or more trajectories
  ros::Time now = ros::Time::now();
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  ////ROS_INFO("Modification trajectories obtained: %i", (int)mod_trajec.size());
  
  Population popCopy = population_;

  ros::Time t_for = ros::Time::now();
  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) 
  {
    ////ROS_INFO("Modified trajectory: %s", mod_trajec.at(i).toString().c_str());
    //std::cout<<"\nramp_planner: Evaluating trajectory "<<(int)i<<"\n";

    // Evaluate the new trajectory
    mod_trajec.at(i) = evaluateTrajectory(mod_trajec.at(i));
    ////ROS_INFO("Done evaluating");

    // Make a copy of the trajec
    RampTrajectory modded_t = mod_trajec.at(i);

    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    // If it was successfully added, push its index onto the result
    ros::Time t_start = ros::Time::now();
    int index = popCopy.add(modded_t);
    
    if(index > -1)
    {
      ////ROS_INFO("Adding trajectory at index %i \n%s", index, mod_trajec[i].toString().c_str());
      ////ROS_INFO("Population Previously: %s", popCopy.toString().c_str());
      popCopy.replace(index, modded_t);
      result.i_modified_.push_back(index);
    }

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) 
    {
      popCopy.createSubPopulations();
      //trans_popCopy.createSubPopulations();
    }
  } // end for

  ros::Time t_p = ros::Time::now();
  result.popNew_    = popCopy;
  //////ROS_INFO("After modification, pop now: %s", result.popNew_.toString().c_str());

  ////ROS_INFO("Exiting Planner::modification");
  return result;
} // End modification




void Planner::modificationOOP()
{
  ros::Time t_m = ros::Time::now();
  //ROS_INFO("In Planner::modificationOOP()");
  ModificationResult result;

  // Modify 1 or more trajectories
  ros::Time now = ros::Time::now();
  std::vector<RampTrajectory> mod_trajec;
  modifyTrajecOOP(mod_trajec);
  ros::Time t_p = ros::Time::now();
  //ROS_INFO("t_p: %f", (t_p-now).toSec());
  //ROS_INFO("Modification trajectories obtained: %i", (int)mod_trajec.size());
  
  if(mod_trajec.size()>1)
    modded_two=true;

  ros::Time t_for = ros::Time::now();
  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) 
  {
    //ROS_INFO("i: %i", i);
    //ROS_INFO("Modified trajectory: %s", mod_trajec.at(i).toString().c_str());
    ////ROS_INFO("Path size: %i", (int)mod_trajec[i].msg_.holonomic_path.points.size());
    //std::cout<<"\nramp_planner: Evaluating trajectory "<<(int)i<<"\n";

    // Evaluate the new trajectory
    //evaluateTrajectoryOOP(mod_trajec[i]);
    
    // Compute full switch (method evaluates the trajectory)
    RampTrajectory traj_final = mod_trajec[i];
    if(cc_started_ && !imminent_collision_)
    {
      computeFullSwitchOOP(movingOn_, mod_trajec[i], controlCycle_.toSec(), traj_final);
    }
    else
    {
      evaluateTrajectoryOOP(traj_final);
    }
    //ROS_INFO("Done evaluating");
    //ROS_INFO("traj_final: %s", traj_final.fitnessFeasibleToString().c_str());

    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    // If it was successfully added, push its index onto the result
    ros::Time t_start = ros::Time::now();
    ////ROS_INFO("Adding to pop, mod_trajec.size(): %i", (int)mod_trajec.size());
    int index = population_.add(traj_final);
    //int index = population_.add(mod_trajec[i]);
    ////ROS_INFO("Done adding to pop");
    
    // No longer need to reset CC time because trajs should have same t_start
    if(index > -1)
    {
      ROS_INFO("Adding trajectory at index %i \n%s", index, traj_final.toString().c_str());
      ////ROS_INFO("Population Previously: %s", popCopy.toString().c_str());
      //controlCycle_ = population_.getEarliestStartTime();
      //controlCycleTimer_.setPeriod(controlCycle_, false);
      num_mods_++;
      if(cc_started_)
      {
      }
      mod_worked=true;
    }

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) 
    {
      population_.createSubPopulations();
      //trans_popCopy.createSubPopulations();
    }
  } // end for

  //////ROS_INFO("After modification, pop now: %s", result.popNew_.toString().c_str());
  ////ROS_INFO("Exiting Planner::modification");
} // End modificationOOP




void Planner::stopForDebugging() 
{

  h_parameters_.setImminentCollision(true); 

  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();
}

void Planner::restartAfterDebugging() 
{
  h_parameters_.setImminentCollision(false); 

  controlCycleTimer_.start();
  planningCycleTimer_.start();
  imminentCollisionTimer_.start();
}

void Planner::pause() 
{
  stopForDebugging();
  std::cout<<"\nPress Enter to continue\n";
  std::cin.get();
  restartAfterDebugging();
}




/** 
 * This method will replace the starting motion state of each path
 * with s and will update the modifier's paths 
 * */
void Planner::updatePathsStart(const MotionState s) 
{
  ////ROS_INFO("In Planner::updatePathsStart");
  
  KnotPoint kp_s(s);

  for(unsigned int i=0;i<population_.paths_.size();i++) {
    population_.paths_.at(i).start_ = s;

    population_.paths_.at(i).msg_.points.erase (population_.paths_.at(i).msg_.points.begin());
    population_.paths_.at(i).msg_.points.insert(population_.paths_.at(i).msg_.points.begin(), kp_s.buildKnotPointMsg());
  }

  ////ROS_INFO("Exiting Planner::updatePathsStart");
} // End updatePathsStart


const RampTrajectory Planner::offsetTrajectory(const RampTrajectory t, const MotionState diff) const
{
  RampTrajectory result = t;  

  result.offsetPositions(diff);

  return result;
}

/*
 * diff will be the amount to offset. Only the positions will be offset
 */
const Population Planner::offsetPopulation(const Population pop, const MotionState diff) const
{
  ////ROS_INFO("In Planner::offsetPopulation");
  Population result = pop;

  ////ROS_INFO("diff: %s", diff.toString().c_str());
 
   
  for(uint8_t i=0;i<pop.size();i++)
  {
    ////ROS_INFO("Trajectory %i", i);
    
    RampTrajectory temp = pop.get(i);
    temp.offsetPositions(diff);
    
    result.replace(i, temp);
  }
   
  ////ROS_INFO("Exiting Planner::offsetPopulation");
  return result;
}


void Planner::offsetTrajectoryOOP(RampTrajectory& t, const MotionState& diff) 
{
  t.offsetPositions(diff);
}

void Planner::offsetPopulationOOP(const MotionState& diff) 
{
  for(uint16_t i=0;i<population_.size();i++)
  {
    population_.trajectories_[i].offsetPositions(diff);
  }
}



const MotionState Planner::errorCorrection()  
{
  ////ROS_INFO("In Planner::errorCorrection");
  MotionState result;

  ////ROS_INFO("c_pc: %i", (int)c_pc_);
  ////ROS_INFO("m_i.size(): %i", (int)m_i_.size());
 
  ////ROS_INFO("m_i[%i]: %s", c_pc_, m_i_.at(c_pc_).toString().c_str());
  ////ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Get the difference between robot's state and what state it should be at
  ros::Duration t_since_cc = ros::Time::now() - t_prevCC_;
  //MotionState diff = m_i_.at(t_since_cc.toSec()).subtractPosition(latestUpdate_, true);
  MotionState diff = movingOnCC_.getPointAtTime(t_since_cc.toSec());
  ////ROS_INFO("Diff before subtract: %s", diff.toString().c_str());
  diff = diff.subtractPosition(latestUpdate_, true);
  MotionState temp = diff_.subtractPosition(diff);
  ////ROS_INFO("Diff after subtract: %s", diff.toString().c_str());
  
  error_correct_val_pos_.push_back( sqrt( pow(temp.msg_.positions.at(0), 2) + pow(temp.msg_.positions.at(1),2) ) );
  error_correct_val_or_.push_back(temp.msg_.positions.at(2));
 
  ////ROS_INFO("m_cc: %s\ndiff: %s", m_cc_.toString().c_str(), diff.toString().c_str());

  //ROS_INFO("temp: %s", temp.toString().c_str());
  //ROS_INFO("m_cc_: %s", m_cc_.toString().c_str());
  // subtractPosition that difference from startPlanning
  result = m_cc_.subtractPosition(temp, true);

  // Set new theta
  result.msg_.positions.at(2) = latestUpdate_.msg_.positions.at(2);


  ////ROS_INFO("result: %s", result.toString().c_str());
  ////ROS_INFO("Exiting Planner::errorCorrection");
  return result;
}




void Planner::planningCycleCallback() 
{
  ros::Time t_start = ros::Time::now();
  //ROS_INFO("*************************************************");
  //ROS_INFO("Planning cycle occurring, generation %i", generation_);
  ////ROS_INFO("  e.last_expected: %f\n  e.last_real: %f\n  current_expected: %f\n  current_real: %f\n  profile.last_duration: %f", e.last_expected.toSec(), e.last_real.toSec(), e.current_expected.toSec(), e.current_real.toSec(), e.profile.last_duration.toSec());
  ////ROS_INFO("Time since last: %f", (e.current_real - e.last_real).toSec());
  //ROS_INFO("*************************************************");
 
  ////ROS_INFO("Time since last CC: %f", (ros::Time::now()-t_prevCC_).toSec());

  EC=false; mod_worked=false; modded_two=false;
 
  MotionState diff;


  /*
   * Error correction
   */
  // Must have started control cycles
  // errorReduction is true
  // Not driving on straight line
  if(errorReduction_ && !imminent_collision_ && cc_started_ && generation_ % 3 == 0 &&
      !(fabs(latestUpdate_.msg_.velocities.at(2)) > 0.1 && sqrt(pow(latestUpdate_.msg_.velocities[0],2) + pow(latestUpdate_.msg_.velocities[1],2)) > 0.01))
  {
    ros::Time t_start_error = ros::Time::now();

    // TODO: Is this if statement needed? Isn't it essentially checking that we are not
    // moving on a curve, which is what the previous one checks?
    // If not first PC and best trajectory has no curve
    // best curve has a curve and we aren't moving on it
    if(population_.getBest().msg_.curves.size() == 0    || 
        ( population_.getBest().msg_.curves.size() > 0  && 
          population_.getBest().msg_.curves.at(0).u_0 < 0.0001))
    {
      ////ROS_INFO("Doing error correction");
      ////ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
      
      // Do error correction
      ros::Duration t_since_cc = ros::Time::now() - t_prevCC_;
      ////ROS_INFO("t_since_cc: %f", t_since_cc.toSec());
      ////ROS_INFO("movingOnCC_: %s", movingOnCC_.toString().c_str());
     
      diff = movingOnCC_.getPointAtTime(t_since_cc.toSec());
      
      ////ROS_INFO("movingOnCC_ at t_since_cc: %s", diff.toString().c_str());
      ////ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());

      diff = diff.subtractPosition(latestUpdate_, true);
      MotionState temp = diff_.subtractPosition(diff);
      ////ROS_INFO("num_cc: %i", (int)num_cc_);
      ////ROS_INFO("diff_: %s diff: %s temp: %s", diff_.toString().c_str(), diff.toString().c_str(), temp.toString().c_str());
      
      diff_ = diff_.subtractPosition(temp);

      ////ROS_INFO("m_cc_: %s", m_cc_.toString().c_str());
      startPlanning_ = m_cc_.add(temp);
      //startPlanning_ = errorCorrection();

      //////ROS_INFO("Updating movingOn");
      movingOn_ = movingOnCC_;
      movingOn_.offsetPositions(temp);

      ////ROS_INFO("Corrected startPlanning_: %s", startPlanning_.toString().c_str());
      ////ROS_INFO("Corrected movingOn_: %s", movingOn_.toString().c_str());

      //population_  = offsetPopulation(population_at_cc_, diff);
      offsetPopulationOOP(temp);

      //population_  = evaluatePopulation(population_);
      evaluatePopulationOOP();

      EC=true;


    } // end if doing error correction
    //else
    //{
      ////ROS_INFO("Not doing error correction");
      ////ROS_INFO("c_pc: %i population_.getBest().msg_.curves.size(): %i", c_pc_, (int)population_.getBest().msg_.curves.size());
      ////ROS_INFO("population_.getBest().msg_.curves.at(0).u_0: %f", population_.getBest().msg_.curves.at(0).u_0); 
    //}

    //error_correct_durs_.push_back(ros::Time::now() - t_start_error);
  } // end if doing error correction 
  /*else
  {
    //ROS_INFO("Not doing error correction");
    //ROS_INFO("cc_started_: %s generation_: %i errorReduction_: %s fabs(latestUpdate_.msg_.velocities.at(2)): %f", 
        cc_started_ ? "True" : "False", generation_,
        errorReduction_ ? "True" : "False", 
        fabs(latestUpdate_.msg_.velocities.at(2)));
  }

  ////ROS_INFO("Done with error correction!");


  /*
   * Modification
   */
  if(modifications_) 
  {
    ////ROS_INFO("*****************************");
    //ROS_INFO("Performing modification");
    ros::Time t = ros::Time::now();
    //ModificationResult mod = modification();
    modificationOOP();
    mutate_durs_.push_back(ros::Time::now() - t);
    //ROS_INFO("Done with modification");
    ////ROS_INFO("*****************************");


    // cc_started needed? still want to replace if they haven't started, only need cc_started when switching 
    // TODO: cc_started used to be a predicate here
    /*if(mod.i_modified_.size() > 0) 
        //&& !population_.get(0).path_.at(0).motionState_.equals(goal_))
    {
      ////ROS_INFO("In if trajectory added");
      population_       = mod.popNew_;
      
      controlCycle_ = population_.getEarliestStartTime();
      controlCycleTimer_.setPeriod(controlCycle_, false);
      //controlCycle_ = population_.getBest().msg_.t_start;
      //controlCycleTimer_.setPeriod(population_.getBest().msg_.t_start, false);
      ////ROS_INFO("Modification: new CC timer: %f", population_.getBest().msg_.t_start.toSec());
    } // end if trajectory added*/
    /*else
    {
      //ROS_INFO("No trajectory added");
    }*/
  } // end if modifications



  /* 
   * Finish up
   */
  // t=t+1
  generation_++;
  c_pc_++;

  //if(cc_started_)
  //{
    //sendPopulation(population_);
  //}
  //else
  //{
  //}
  ////ROS_INFO("population.bestID: %i", population_.calcBestIndex());

  ////ROS_INFO("Pop: %s", population_.toString().c_str());
  /*////ROS_INFO("Exiting PC at time: %f", ros::Time::now().toSec());
  ////ROS_INFO("Time spent in PC: %f", (ros::Time::now() - t).toSec());*/


  sendPopulation(population_);
  
  ros::Duration d = ros::Time::now() - t_start; 
  //ROS_INFO("d: %f EC: %s mod_worked: %s modded_two: %s", d.toSec(), EC ? "True" : "False", mod_worked ? "True" : "False", modded_two ? "True" : "False");
  pc_durs_.push_back(d);
  ////ROS_INFO("********************************************************************");
  ////ROS_INFO("Generation %i completed", (generation_-1));
  ////ROS_INFO("********************************************************************");
} // End planningCycleCallback




const uint8_t Planner::computeSwitchPC(const RampTrajectory target, const RampTrajectory moving)
{
  ////ROS_INFO("In Planner::computeSwitchPC(RampTrajectory, RampTrajectory)");
  // Worst case - no curve possible, stop and rotate at next CC
  int result = generationsPerCC_;
  ////ROS_INFO("moving: %s", moving.toString().c_str());

  for(int i_pc=result-1;i_pc>0;i_pc--)
  {
    ////ROS_INFO("i_pc: %i", i_pc);

    // Get t and the motion state along moving at time t
    double t = i_pc*planningCycle_.toSec();
    MotionState ms = moving.getPointAtTime(t);

    // Must be moving straight
    // TODO: Change this to oriented towards end of moving
    if(fabs(ms.msg_.velocities.at(2)) < 0.001)
    {
      RampTrajectory trans = getTransitionTrajectory(moving, target, t);

      ////ROS_INFO("trans: %s", trans.toString().c_str());

      // If we are able to plan a curve and it's not the best
      if( (trans.msg_.i_knotPoints.size() == 2) ||
          (trans.msg_.curves.size() > 0 &&
          trans.msg_.i_knotPoints.size() < 5) )
      {
        ////ROS_INFO("Able to plan a transition curve!");
      } // end if
      else
      {
        ////ROS_INFO("Unable to plan a transition curve!");
        ////ROS_INFO("trans.msg_.curves.size(): %i", (int)trans.msg_.curves.size());
        ////ROS_INFO("trans.msg_.i_knotPoints.size(): %i", (int)trans.msg_.i_knotPoints.size());
      }
    } // end if moving on straight line
    else
    {
      ////ROS_INFO("Stopping! Not moving on straight line at pc: %i", i_pc);
      i_pc = 0;
    }
  }

  ////ROS_INFO("Exiting Planner::computeSwitchPC(RampTrajectory, RampTrajectory)");
  return result;
} // End computeSwitchPC



const uint8_t Planner::computeSwitchPC(const Population pop, const RampTrajectory moving) 
{
  ////ROS_INFO("In Planner::computeSwitchPC()");

  int result = generationsPerCC_;
  //////ROS_INFO("moving: %s", moving.toString().c_str());

  for(int i_pc=result-1;i_pc>0;i_pc--)
  {
    //////ROS_INFO("i_pc: %i", i_pc);
    double t = i_pc*planningCycle_.toSec();

    // If the robot is moving on a straight line at this pc, compute curve
    MotionState ms = moving.getPointAtTime(t);
    //////ROS_INFO("ms at i_pc %i: %s", i_pc, ms.toString().c_str());
    if(fabs(ms.msg_.velocities.at(2)) < 0.001)
    {

      // Go through each trajectory and try to get a transition curve
      for(uint8_t i=0;i<pop.size();i++)
      {
        //////ROS_INFO("Inner for, i: %i", i);
        RampTrajectory T_new = getTransitionTrajectory(moving, pop.get(i), t);
        //////ROS_INFO("T_new: %s", T_new.toString().c_str());

        // If we are able to plan a curve
        if( (T_new.msg_.i_knotPoints.size() == 2) ||
            ( T_new.msg_.curves.size() > 0 &&
              T_new.msg_.i_knotPoints.size() < 5) )
        {
          ////ROS_INFO("Able to plan a transition curve!");
        } // end if
        else
        {
          ////ROS_INFO("Unable to plan a transition curve!");
          ////ROS_INFO("T_new.msg_.curves.size(): %i", (int)T_new.msg_.curves.size());
          ////ROS_INFO("T_new.msg_.i_knotPoints.size(): %i", (int)T_new.msg_.i_knotPoints.size());
          i=pop.size();
        }
      } // end for
    } // end if moving on straight line
    else
    {
      ////ROS_INFO("Stopping! Not moving on straight line at pc: %i", i_pc);
      i_pc = 0;
    }
  } // end for

  //////ROS_INFO("Result: %i", result);
  ////ROS_INFO("Exiting Planner::computeSwitchPC");
  return result;
}



double Planner::getEarliestStartTime(const RampTrajectory& from)
{
  //ROS_INFO("In Planner::getEarliestStartTime");
  //ROS_INFO("From: %s", from.toString().c_str());
  double result;

  std::vector<double> times;
  for(int i=0;i<population_.size();i++)
  {
    RampTrajectory traj = population_.get(i);

    uint8_t deltasPerCC = (t_fixed_cc_+0.0001) / delta_t_switch_;
    uint8_t delta_t_now = (ros::Time::now() - t_prevCC_).toSec() / delta_t_switch_;
    double  delta_t     = ((deltasPerCC+1)*delta_t_switch_);
    for(int i_delta_t=deltasPerCC-1; i_delta_t > (delta_t_now+1); i_delta_t--)
    {
      double t = i_delta_t * delta_t_switch_;

      MotionState ms = traj.getPointAtTime(t);
      if(predictTransition(from, traj, t))
      {
        times.push_back(t); 
        break;
      } // end if
    } // end for starting times
    // what if no switch
  } // end for each trajectory

  if(times.size() == 0)
  {
    result = t_fixed_cc_;
  }
  else
  {
    // Get earliest time
    result = times[0];
    for(int i=1;i<times.size();i++)
    {
      if(times[i] < result)
      {
        result = times[i];
      }
    } // end for
  } // end else

  //ROS_INFO("Exiting Planner::getEarliestStartTime");
  return result;
} // End getEarliestStartTime



void Planner::computeFullSwitchOOP(const RampTrajectory& from, const RampTrajectory& to, const double& t_start, RampTrajectory& result)
{
  //ROS_INFO("In Planner::computeFullSwitchOOP");
  ////ROS_INFO("to: %s", to.toString().c_str());

  // Get transition trajectory
  ros::Time tt = ros::Time::now();
  std::vector<RampTrajectory> trajecs;
  switchTrajectoryOOP(from, to, t_start, trajecs);
  //////ROS_INFO("Time spent getting switch trajectory: %f", (ros::Time::now()-tt).toSec());
  
  //ROS_INFO("trajecs.size(): %i", (int)trajecs.size());

  // If a switch was possible
  if(trajecs.size() > 0)
  {
    //ROS_INFO("Switch was possible");
    RampTrajectory T_new  = trajecs.at(1);
    Path p                = T_new.msg_.holonomic_path;
    //////ROS_INFO("Before eval, T_new.path: %s", T_new.path_.toString().c_str());
    //ROS_INFO("to.msg.holonomic_path: %s", utility_.toString(to.msg_.holonomic_path).c_str());;
    
    //ROS_INFO("T_new.fitness before: %f", T_new.msg_.fitness);

    // Evaluate T_new
    //ramp_msgs::EvaluationRequest er = buildEvaluationRequest(T_new);
    requestEvaluationOOP(T_new);
    //ROS_INFO("T_new.fitness after: %f", T_new.msg_.fitness);

    // Set misc members
    if(trajecs[0].msg_.curves.size() > 0 || trajecs[0].msg_.i_knotPoints.size() == 2)
    {
      T_new.transitionTraj_ = trajecs.at(0).msg_;
    }
    T_new.msg_.holonomic_path = p.msg_;

    // Set result
    result                  = T_new;
    //ROS_INFO("result: %s", result.toString().c_str());
    //////ROS_INFO("result.transitionTraj.size(): %i", (int)result.transitionTraj_.trajectory.points.size());

    //////ROS_INFO("After eval, T_new.path: %s", T_new.path_.toString().c_str());
  }

  // If a switch was not possible, just return
  // the holonomic trajectory
  else
  {
    //ROS_WARN("A switch was not possible, returning \"to\" trajectory: %s", to.toString().c_str());
    result = to;
  }

  ////ROS_INFO("Full switch result: %s", result.toString().c_str());
  //ROS_INFO("Exiting Planner::computeFullSwitchOOP");
}
    
void Planner::switchTrajectoryOOP(const RampTrajectory& from, const RampTrajectory& to, const double& t_start, std::vector<RampTrajectory>& result)
{

  /*
   * Call getTransitionTrajectory
   * if we can find one before next CC
   */
  //if(delta_t  < ((deltasPerCC+1)*delta_t_switch_))
  //{
  RampTrajectory switching, full;
  getTransitionTrajectoryOOP(from, to, t_start, switching);
  full = switching;
  ////ROS_INFO("Switching trajectory: %s", switching.toString().c_str());

  // If robot is at goal, full should only be 1 point,
  // check for this to prevent crashing
  if(full.msg_.i_knotPoints.size() > 1)
  {
    // Keep a counter for the knot points
    // Start at 1 because that should be the starting knot point of the curve
    int c_kp = 1;
    
    ////ROS_INFO("c_kp: %i", c_kp);
    ////ROS_INFO("c_kp: %i i_knotPoints.size(): %i", c_kp, (int)to.msg_.i_knotPoints.size());
    
    ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());

    // Set full as the concatenating of switching and to
    full        = switching.concatenate(to, c_kp);
    
    ////ROS_INFO("full.path: %s", full.msg_.holonomic_path.toString().c_str());


    // Set the proper ID, path, and t_starts
    full.msg_.id              = to.msg_.id;
    full.msg_.holonomic_path  = to.msg_.holonomic_path;

    full.msg_.t_start       = ros::Duration(t_start);
    switching.msg_.t_start  = full.msg_.t_start;

    if(full.transitionTraj_.curves.size() > 0 || full.transitionTraj_.i_knotPoints.size() == 2)
    {
      full.transitionTraj_    = switching.msg_;
    }

    result.push_back(switching);
    result.push_back(full);
  } // end if size > 1
  //} // end if switch possible
 

  if(print_enter_exit_)
  {
    //ROS_INFO("Exiting Planner::switchTrajectoryOOP");
  }
}



void Planner::getTransPopOOP(const Population& pop, const RampTrajectory& movingOn, const double& t_start, Population& result)
{
  //ROS_INFO("In Planner::getTransPopOOP");
  //////ROS_INFO("pop: %s", pop.toString().c_str());
  result = pop;

  if(result.type_ != HOLONOMIC)
  {
    // Go through the population and get:
    // 1) planning cycle to switch at
    // 2) transition trajectory
    for(uint8_t i=0;i<pop.size();i++)
    {
      ////ROS_INFO("i: %i", i);
      RampTrajectory temp;
      computeFullSwitchOOP(movingOn_, pop.get(i), t_start, temp);
      result.replace(i, temp);
    }
  }
  //////ROS_INFO("Trans pop full: %s", result.toString().c_str());

  //ROS_INFO("Exiting Planner::getTransPopOOP");
}

void Planner::getTransPopOOP(const Population& pop, const RampTrajectory& movingOn, Population& result)
{
  //ROS_INFO("In Planner::getTransPopOOP");
  //////ROS_INFO("pop: %s", pop.toString().c_str());
  result = pop;

  if(result.type_ != HOLONOMIC)
  {
    // Go through the population and get:
    // 1) planning cycle to switch at
    // 2) transition trajectory
    for(uint8_t i=0;i<pop.size();i++)
    {
      ////ROS_INFO("i: %i", i);
      RampTrajectory temp;
      computeFullSwitchOOP(movingOn_, pop.get(i), temp);
      result.replace(i, temp);
    }
  }
  //////ROS_INFO("Trans pop full: %s", result.toString().c_str());

  //ROS_INFO("Exiting Planner::getTransPopOOP");
} // End getTransPopOOP



const Population Planner::getTransPop(const Population pop, const RampTrajectory movingOn)
{
  ////ROS_INFO("In Planner::getTransPop");
  //////ROS_INFO("pop: %s", pop.toString().c_str());
  Population result = pop;

  if(result.type_ != HOLONOMIC)
  {
    // Go through the population and get:
    // 1) planning cycle to switch at
    // 2) transition trajectory
    for(uint8_t i=0;i<pop.size();i++)
    {
      ////ROS_INFO("i: %i", i);
      RampTrajectory temp = computeFullSwitch(movingOn_, pop.get(i));
      result.replace(i, temp);
    }
  }
  //////ROS_INFO("Trans pop full: %s", result.toString().c_str());

  ////ROS_INFO("Exiting Planner::getTransPop");
  return result;
}







/** This methed runs the tasks needed to do a control cycle */
void Planner::doControlCycle() 
{
  ////ROS_WARN("Control Cycle %i occurring at Time: %f", num_cc_, ros::Time::now().toSec());
  ROS_INFO("controlCycle_: %f", controlCycle_.toSec());
  //ROS_INFO("Time between control cycles: %f", (ros::Time::now() - t_prevCC_).toSec());
  t_prevCC_ = ros::Time::now();
  ////ROS_INFO("Number of planning cycles that occurred between CC's: %i", c_pc_);

  ros::Time t = ros::Time::now();

  // Set all of the trajectory t_start values to 0 b/c they would be starting now
  for(int i=0;i<population_.size();i++)
  {
    population_.trajectories_[i].msg_.t_start = ros::Duration(0);
  }

  //population_ = evaluatePopulation(population_);
  evaluatePopulationOOP();

  // Set the bestT
  RampTrajectory bestT = population_.getBest();

  ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());

  // Send the best trajectory and set movingOn
  //////ROS_INFO("Sending best");
  ROS_INFO("bestT: %s", bestT.toString().c_str());
  sendBest();
  //////ROS_INFO("After sendBest");


  //////ROS_INFO("Setting movingOn_");
  movingOnCC_             = bestT.getSubTrajectory(t_fixed_cc_);
  movingOnCC_.msg_.curves = bestT.msg_.curves;
  movingOnCC_.msg_.t_start  = ros::Duration(0);

  // Evaluate movingOnCC
  ROS_INFO("Evaluting movingOnCC");
  evaluateTrajectoryOOP(movingOnCC_, false);
  //movingOnCC_               = evaluateTrajectory(movingOnCC_);
  
  //ROS_INFO("movingOnCC_: %s", movingOnCC_.toString().c_str());
  ////ROS_INFO("Evaluating movingOn in CC");
  
  movingOn_               = movingOnCC_;
  moving_on_coll_         = !movingOn_.msg_.feasible;
  ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
  
  ROS_INFO("movingOn_.t_firstCollision: %f", movingOn_.msg_.t_firstCollision.toSec());
  if(!(ob_trajectory_.size() > 0 && moving_on_coll_ && (movingOn_.msg_.t_firstCollision.toSec() < controlCycle_.toSec()
    || (movingOn_.msg_.t_firstCollision.toSec() - (ros::Time::now().toSec()-t_prevCC_.toSec())) < controlCycle_.toSec())))
  {
    ROS_INFO("No IC");
    imminent_collision_ = false;
  }
  else
  {
    ROS_INFO("IC Detected in CC Callback");
  }


  // The motion state that we should reach by the next control cycle
  if(imminent_collision_)
  {
    ROS_INFO("imminent_collision_: True");
    m_cc_ = latestUpdate_;
    startPlanning_ = m_cc_;
    controlCycle_ = ros::Duration(t_fixed_cc_);
    sendPopulation(population_);
    reset_ = false;

    // Set all of the trajectory t_start values to 0 b/c they would be starting now
    for(int i=0;i<population_.size();i++)
    {
      population_.trajectories_[i].msg_.t_start = ros::Duration(t_fixed_cc_);
    }
    ROS_INFO("population_: %s", population_.toString().c_str());
  }
  // If trajectory does not reach t_fixed_cc, last point will be returned
  else
  {
    ROS_INFO("Setting ic.data = false and imminent_collision = false");
    std_msgs::Bool ic;
    ic.data = false;
    h_control_->sendIC(ic);
    imminent_collision_ = false;
    m_cc_ = bestT.getPointAtTime(t_fixed_cc_);
    reset_ = false;
  

  // At CC, startPlanning is assumed to be perfect (no motion error accounted for yet)
  startPlanning_ = m_cc_;
  //ROS_INFO("New startPlanning_: %s", startPlanning_.toString().c_str());

  //ROS_INFO("Before adaptation and evaluation, pop size: %i", population_.size());
  for(uint8_t i=0;i<populationSize_;i++)
  {
    //ROS_INFO("Trajectory %i: %s", i, population_.get(i).toString().c_str());
  }
  ////ROS_INFO("transPop.bestID: %i", population_.calcBestIndex());

  // Adapt and evaluate population
  ////ROS_INFO("About to adapt, controlCycle_: %f", controlCycle_.toSec());
 
  ros::Time t_startAdapt = ros::Time::now();
  
  // Adapt population
  // Check if bestT time reaches t_fixed_cc
  if(bestT.getT() <= t_fixed_cc_)
  {
    adaptPopulationOOP(startPlanning_, ros::Duration(t_fixed_cc_));
  }
  else
  {
    adaptPopulationOOP(startPlanning_, ros::Duration(bestT.getT()));
  }

  // Why do this here? Non-hybrids are replaced by hybrids and 
  // hybrids are evaluated after created
  evaluatePopulationOOP();

  ros::Duration d_adapt = ros::Time::now() - t_startAdapt;
  adapt_durs_.push_back(d_adapt);

  if(utility_.positionDistance(movingOn_.msg_.trajectory.points.at(movingOn_.msg_.trajectory.points.size()-1).positions,
        population_.getBest().msg_.trajectory.points.at(0).positions) > 0.01)
  {
    //ROS_WARN("Moving on and next best DO NOT MATCH: %f", utility_.positionDistance(movingOn_.msg_.trajectory.points.at(movingOn_.msg_.trajectory.points.size()-1).positions, population_.getBest().msg_.trajectory.points.at(0).positions));
  
    //ROS_WARN("movingOn_: %s", movingOn_.toString().c_str());
    //ROS_WARN("Pop best: %s", population_.getBest().toString().c_str());
  }
  
  ROS_INFO("After adaptation and evaluation:");
  ROS_INFO("Pop earliest time: %f", population_.getEarliestStartTime().toSec());
  for(int i=0;i<population_.size();i++)
  {
    ROS_INFO("%s", population_.get(i).toString().c_str());
  }
  //////ROS_INFO("Time spent adapting: %f", d_adapt.toSec());
 
  //if(population_.calcBestIndex() != population_.calcBestIndex())
  //{
    ////ROS_WARN("Population (holo) best ID: %i\nPopulation (non-holo) best ID: %i", population_.calcBestIndex(), population_.calcBestIndex());
    //////ROS_INFO("Pop: %s", population_.toString().c_str());
    //////ROS_INFO("Trans Pop: %s", population_.toString().c_str());
  //}


  ////ROS_INFO("Before getTransPop: %s", population_.toString().c_str());
 
  // Find the transition (non-holonomic) population and set new control cycle time
  ros::Time t_startTrans = ros::Time::now();

  double t_start = getEarliestStartTime(movingOn_);
  //ROS_INFO("t_start: %f", t_start);
  getTransPopOOP(population_, movingOn_, t_start, population_);
 
  //getTransPopOOP(population_, movingOn_, population_);
  //ROS_INFO("Evaluating transPop");
  evaluatePopulationOOP();
  
  ros::Duration d_trans = ros::Time::now() - t_startTrans;
  trans_durs_.push_back(d_trans);
  
  ////ROS_INFO("After finding transition population, controlCycle period: %f", controlCycle_.toSec());
  ROS_INFO("New transPop:"); 
  ROS_INFO("Pop earliest time: %f", population_.getEarliestStartTime().toSec());
  for(int i=0;i<population_.size();i++)
  {
    ROS_INFO("%s", population_.get(i).toString().c_str());
  }
  ////ROS_INFO("Time spent getting trans pop: %f", d_trans.toSec());

  population_at_cc_  = population_;
  diff_.zero();


  // If error reduction
  // Set pop_orig_ and totalDiff to 0's
  if(errorReduction_) 
  {
    m_i_ = setMi(movingOn_);
  }
 
  // Create sub-populations if enabled
  if(subPopulations_) 
  {
    population_.createSubPopulations();
  }


  // End if no imminent collision
  }
 
  // Send the population to trajectory_visualization
  sendPopulation(population_);
  
  controlCycle_         = population_.getEarliestStartTime();
  controlCycleTimer_.setPeriod(controlCycle_, false);

  ROS_INFO("Next CC Time: %f", controlCycle_.toSec());

  ros::Duration d_cc = ros::Time::now() - t;
  cc_durs_.push_back(d_cc);
 
  num_cc_++;
  ////ROS_INFO("Time spent in CC: %f", d_cc.toSec());
  ////ROS_INFO("Exiting Planner::doControlCycle");
} // End doControlCycle





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent& e) 
{
  // Restart the planning cycles
  //planningCycleTimer_.stop();
  //planningCycleTimer_.start();
  
  /*//ROS_INFO("*************************************************");
  //ROS_INFO("  Control cycle timer event happening  ");
  //ROS_INFO("  e.last_expected: %f\n  e.last_real: %f\n  current_expected: %f\n  current_real: %f\n  profile.last_duration: %f",
      e.last_expected.toSec(), e.last_real.toSec(), e.current_expected.toSec(), e.current_real.toSec(), e.profile.last_duration.toSec());
  //ROS_INFO("Time since last: %f", (e.current_real - e.last_real).toSec());
  //ROS_INFO("*************************************************");*/
  
  ////ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Do the control cycle
  doControlCycle();

  //ros::WallDuration diff = controlCycle_ - e.profile.last_duration;
  
  //ros::NodeHandle h; 
  //controlCycleTimer_ = h.createWallTimer(controlCycle_, &Planner::controlCycleCallback, this)
  //e.profile.last_duration = e.profile.last_duration - diff;
  
  // Set flag showing that CCs have started
  if(!cc_started_) 
  {
    cc_started_ = true;
    h_parameters_.setCCStarted(true); 
  }
    
  ////ROS_INFO("Leaving Control Cycle, period: %f", controlCycle_.toSec());
} // End controlCycleCallback











/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {
  //////ROS_INFO("Sending best trajectory: %s", population_.get(population_.calcBestIndex()).toString().c_str());

  //if(!stop_) {
    RampTrajectory best = population_.getBest();
    //RampTrajectory best = bestTrajec_;

    // If infeasible and too close to obstacle, 
    // Stop the robot by sending a blank trajectory
    /*if(!best.msg_.feasible && (best.msg_.t_firstCollision.toSec() < 3.f)) 
    {
      //std::cout<<"\nCollision within 3 seconds! Stopping robot!\n";
    }
    else if(!best.msg_.feasible) {
      ////ROS_INFO("Best trajectory is not feasible! Time until collision: %f", best.msg_.t_firstCollision.toSec());
    }*/
    
    best.msg_.header.stamp = ros::Time::now();
    h_control_->send(best.msg_);
  //} // end if not stopped
  /*else {
    ////ROS_INFO("Sending blank!");
    RampTrajectory blank;
    h_control_->send(blank.msg_);
  }*/
} // End sendBest







/** Send the whole population of trajectories to the trajectory viewer */
void Planner::sendPopulation(const Population& pop) const 
{
  ramp_msgs::Population msg;

  /*if(subPopulations_) 
  {
    Population temp(pop.getNumSubPops());
    std::vector<RampTrajectory> trajecs = pop.getBestFromSubPops();
    for(uint8_t i=0;i<trajecs.size();i++) 
    {
      temp.add(trajecs.at(i));
    }

    temp.calcBestIndex();
    msg = temp.populationMsg();
  }*/
  //else 
  //{
    msg = pop.populationMsg();
  //}
  /*for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    msg.population.push_back(ob_trajectory_.at(i).msg_);
  }*/

  msg.robot_id = id_;

  msg.population.push_back(movingOn_.msg_);
  h_control_->sendPopulation(msg);
}

void Planner::displayTrajectory(const ramp_msgs::RampTrajectory traj) const 
{
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  h_control_->sendPopulation(pop);
}





/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
const RampTrajectory Planner::evaluateTrajectory(const RampTrajectory trajec) 
{
  ////ROS_INFO("In Planner::evaluateTrajectory");

  RampTrajectory result = requestEvaluation(trajec);
  //////ROS_INFO("result: %s", result.toString().c_str());

  ////ROS_INFO("Leaving Planner::evaluateTrajectory");
  return result;
} // End evaluateTrajectory



/** 
 * This method evaluates each trajectory in the population
 * It also sets i_best_prev_
 **/
const Population Planner::evaluatePopulation(const Population pop) 
{
  ////ROS_INFO("In Planner::evaluatePopulation");
  Population result = pop;
  ////ROS_INFO("Before evaluating, pop: %s", pop.fitnessFeasibleToString().c_str());
  
  std::vector<RampTrajectory> evaled_pop = requestEvaluation(pop.getTrajectories());
  result.replaceAll(evaled_pop);

  // Go through each trajectory in the population and evaluate it
  /*for(uint16_t i=0;i<result.size();i++) 
  {
    //////ROS_INFO("i: %i", (int)i);
    result.replace(i, evaled_pop.at(i));
  } // end for*/
  ////ROS_INFO("After evaluating, pop: %s", result.fitnessFeasibleToString().c_str());
 

  i_best_prev_ = result.calcBestIndex();
  
  ////ROS_INFO("Exiting Planner::evaluatePopulation");
  return result;
} // End evaluatePopulation



void Planner::requestEvaluationOOP(std::vector<RampTrajectory>& trajecs) 
{
  ramp_msgs::EvaluationSrv srv;
  buildEvaluationSrvOOP(trajecs, srv);

  ros::Time t_start = ros::Time::now();
  if(h_eval_req_->request(srv))
  {
    eval_durs_.push_back( ros::Time::now() - t_start );
    if(eval_durs_[eval_durs_.size()-1].toSec() > 0.01)
    {
      //ROS_INFO("(Planner) Long Eval Request (%i total evals): ", (int)trajecs.size());
      /*for(int i=0;i<trajecs.size();i++)
      {
        ROS_INFO("trajec i: %s", trajecs[i].toString().c_str());
      }*/
    }
    for(uint16_t i=0;i<trajecs.size();i++)
    {
      trajecs[i].msg_.fitness          = srv.response.resps[i].fitness;
      trajecs[i].msg_.feasible         = srv.response.resps[i].feasible;
      trajecs[i].msg_.t_firstCollision = srv.response.resps[i].t_firstCollision;
    }
  }
  else
  {
    //ROS_ERROR("An error occurred when evaluating a trajectory");
  }
}


void Planner::requestEvaluationOOP(ramp_msgs::EvaluationRequest& request) const
{
  //ROS_INFO("In Planner::requestEvaluationOOP(EvaluationRequest&)");
  ramp_msgs::EvaluationSrv srv;
  srv.request.reqs.push_back(request);

  if(h_eval_req_->request(srv))
  {
    //ROS_INFO("Setting fitness: %f", srv.response.resps[0].fitness);
    request.trajectory.fitness          = srv.response.resps[0].fitness;
    request.trajectory.feasible         = srv.response.resps[0].feasible;
    request.trajectory.t_firstCollision = srv.response.resps[0].t_firstCollision;
  }
  else
  {
    //ROS_ERROR("An error occurred when evaluating a trajectory");
  }
  //ROS_INFO("Exiting Planner::requestEvaluationOOP(EvaluationRequest&)");
}



void Planner::requestEvaluationOOP(RampTrajectory& trajec, bool full) const
{
  //ROS_INFO("In Planner::requestEvaluationOOP(RampTrajectory&, bool)");
  //ROS_INFO("full: %s", full ? "True" : "False");
  ramp_msgs::EvaluationRequest req;
  
  buildEvaluationRequestOOP(trajec, req, full);
  requestEvaluationOOP(req);
  
  trajec.msg_.fitness           = req.trajectory.fitness;
  trajec.msg_.feasible          = req.trajectory.feasible;
  trajec.msg_.t_firstCollision  = req.trajectory.t_firstCollision;
  //ROS_INFO("trajec.fitness: %f", trajec.msg_.fitness);
  //ROS_INFO("Exiting Planner::requestEvaluationOOP(RampTrajectory&, bool)");
}





void Planner::evaluateTrajectoryOOP(RampTrajectory& t, bool full) const
{
  requestEvaluationOOP(t, full);
}


void Planner::evaluatePopulationOOP()
{
  requestEvaluationOOP(population_.trajectories_);
  /*for(uint16_t i=0;i<population_.size();i++)
  {
    evaluateTrajectory(population_.trajectories_[i]);
  }*/
}













const std::string Planner::pathsToString() const {
  std::ostringstream result;

  result<<"\nPaths:";
  for(unsigned int i=0;i<population_.paths_.size();i++) {
    result<<"\n  "<<population_.paths_.at(i).toString();
  }
  result<<"\n";
  return result.str();
}




const MotionState Planner::findAverageDiff() {
  MotionState result(SP_LU_diffs_.at(0));

  for(uint16_t i=1;i<SP_LU_diffs_.size();i++) {
    result = result.add(SP_LU_diffs_.at(i).abs());
  }

  result = result.divide(SP_LU_diffs_.size());

  return result;
}


void Planner::reportData() 
{
  double sum = 0.;
  for(uint16_t i=0;i<adapt_durs_.size();i++)
  {
    ROS_INFO("Adaptation duration[i]: %f", adapt_durs_.at(i).toSec());
    sum += adapt_durs_.at(i).toSec();
  }
  avg_adapt_dur_ = sum / adapt_durs_.size();
  ROS_INFO("Average adaptation duration: %f", avg_adapt_dur_);


  sum = 0.;
  for(uint16_t i=0;i<trans_durs_.size();i++)
  {
    ROS_INFO("transition duration[i]: %f", trans_durs_.at(i).toSec());
    sum += trans_durs_.at(i).toSec();
  }
  avg_trans_dur_ = sum / trans_durs_.size();
  ROS_INFO("Average transition duration: %f", avg_trans_dur_);

  sum = 0.;
  for(uint16_t i=0;i<cc_durs_.size();i++)
  {
    ROS_INFO("cc duration[i]: %f", cc_durs_.at(i).toSec());
    sum += cc_durs_.at(i).toSec();
  }
  avg_cc_dur_ = sum / cc_durs_.size();
  ROS_INFO("Average cc duration: %f", avg_cc_dur_);

  sum = 0.;
  for(uint16_t i=0;i<mutate_durs_.size();i++)
  {
    ROS_INFO("mutate duration[i]: %f", mutate_durs_.at(i).toSec());
    sum += mutate_durs_.at(i).toSec();
  }
  avg_mutate_dur_ = sum / mutate_durs_.size();
  ROS_INFO("Average mutate duration: %f", avg_mutate_dur_);

  sum = 0.;
  for(uint16_t i=0;i<error_correct_durs_.size();i++)
  {
    ROS_INFO("error correct duration[i]: %f", error_correct_durs_.at(i).toSec());
    sum += error_correct_durs_.at(i).toSec();
  }
  avg_error_correct_dur_ = sum / error_correct_durs_.size();
  ROS_INFO("Average error_correct duration: %f", avg_error_correct_dur_);

  sum = 0.;
  for(uint16_t i=0;i<pc_durs_.size();i++)
  {
    ROS_INFO("pc duration[i]: %f", pc_durs_.at(i).toSec());
    sum += pc_durs_.at(i).toSec();
  }
  avg_pc_dur_ = sum / pc_durs_.size();
  ROS_INFO("Average pc duration: %f", avg_pc_dur_);


  sum = 0.;
  for(uint16_t i=0;i<sc_durs_.size();i++)
  {
    ROS_INFO("sc duration[i]: %f", sc_durs_.at(i).toSec());
    sum += sc_durs_.at(i).toSec();
  }
  avg_sc_dur_ = sum / sc_durs_.size();
  ROS_INFO("Average sc duration: %f", avg_sc_dur_);


  sum = 0.;
  for(uint16_t i=0;i<trajec_durs_.size();i++)
  {
    if(i % 5 == 0)
    {
      ROS_INFO("trajec duration[%i]: %f", i, trajec_durs_.at(i).toSec());
    }
    sum += trajec_durs_.at(i).toSec();
  }
  avg_trajec_dur_ = sum / trajec_durs_.size();
  ROS_INFO("Average trajec duration: %f", avg_trajec_dur_);


  sum = 0.;
  for(uint16_t i=0;i<eval_durs_.size();i++)
  {
    if(i % 20 == 0)
    {
      ROS_INFO("eval duration[i]: %f", eval_durs_.at(i).toSec());
    }
    sum += eval_durs_.at(i).toSec();
  }
  avg_eval_dur_ = sum / eval_durs_.size();
  ROS_INFO("Average eval duration: %f", avg_eval_dur_);


  sum = 0.;
  for(uint16_t i=0;i<error_correct_val_pos_.size();i++)
  {
    ROS_INFO("error_correct_pos[i]: %f", error_correct_val_pos_.at(i));
    sum += error_correct_val_pos_.at(i);
  }
  avg_error_correct_val_pos_ = sum / error_correct_val_pos_.size();
  ROS_INFO("Average position error: %f", avg_error_correct_val_pos_);


  sum = 0.;
  for(uint16_t i=0;i<error_correct_val_or_.size();i++)
  {
    ROS_INFO("error_correct_or[i]: %f", error_correct_val_or_.at(i));
    sum += error_correct_val_or_.at(i);
  }
  avg_error_correct_val_or_ = sum / error_correct_val_or_.size();
  ROS_INFO("Average orientation error: %f", avg_error_correct_val_or_);


  ROS_INFO("num_mods_: %i", num_mods_);
}


/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


void Planner::go() 
{

  // t=0
  generation_ = 0;
  
  // initialize population
  initPopulation();
  evaluatePopulationOOP();
  //ROS_INFO("Population Initialized");
  std::cin.get();
  sendPopulation(population_);
  std::cout<<"\ntransPopulation initialized! Press enter to continue\n";
  //std::cin.get();
 


  if(seedPopulation_) 
  {
    std::cout<<"\nSeeding transPopulation\n";
    seedPopulation();
    i_best_prev_ = population_.calcBestIndex();
    std::cout<<"\ntransPopulation seeded!\n";
    std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";
    std::cout<<"\n** transPop **:"<<population_.toString();

    // Evaluate after seeding
    population_ = evaluatePopulation(population_);

    // Set movingOn
    movingOn_ = population_.get(population_.calcBestIndex()).getSubTrajectory(controlCycle_.toSec());
    ////ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
    

    sendPopulation(population_);
    std::cout<<"\ntransPopulation seeded! Press enter to continue\n";
    std::cin.get();
  }


  // Create sub-transPops if enabled
  if(subPopulations_) 
  {
    population_.createSubPopulations();
    std::cout<<"\nSub-transPopulations created\n";
  }


  // Initialize transtransPopulation
  population_       = population_;
  population_at_cc_ = population_;

  t_start_ = ros::Time::now();

  //ROS_INFO("Planning Cycles started!");

  // Start the planning cycles
  planningCycleTimer_.start();
    
  
  h_parameters_.setCCStarted(false); 


  int num_pc = generationsBeforeCC_; 
  if(num_pc < 0)
  {
    ////ROS_WARN("num_pc is less than zero: %i - Setting num_pc = 0", num_pc);
    num_pc = 0;
  }
  //ROS_INFO("generationsBeforeCC_: %i generationsPerCC_: %i num_pc: %i", generationsBeforeCC_, generationsPerCC_, num_pc);

  ros::Rate r(20);
  // Wait for the specified number of generations before starting CC's
  while(generation_ < num_pc) {planningCycleCallback(); r.sleep(); ros::spinOnce();}
 
  //ROS_INFO("Starting CCs at t: %f", ros::Time::now().toSec());

  // Right before starting CC, make sure transtransPopulation is updated
  population_ = population_;

  diff_ = diff_.zero(3);
  
  // Start the control cycles
  controlCycleTimer_.start();
  imminentCollisionTimer_.start();
  ob_dists_timer_.start();

  ////ROS_INFO("CCs started");

 
  // Do planning until robot has reached goal
  // D = 0.4 if considering mobile base, 0.2 otherwise
  ros::Time t_start = ros::Time::now();
  goalThreshold_ = 0.5;
  while( (latestUpdate_.comparePosition(goal_, false) > goalThreshold_) && ros::ok()) 
  {
    planningCycleCallback();
    r.sleep();
    ros::spinOnce(); 
  } // end while
  ros::Duration t_execution = ros::Time::now() - t_start;
  reportData();
  //ROS_INFO("Total execution time: %f", t_execution.toSec());

  ////ROS_INFO("Planning done!");
  ////ROS_INFO("latestUpdate_: %s\ngoal: %s", latestUpdate_.toString().c_str(), goal_.toString().c_str());
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();
  ob_dists_timer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::RampTrajectory empty;
  h_control_->send(empty);
  h_control_->send(empty);
  h_control_->send(empty);
 
 
  ////ROS_INFO("Total number of planning cycles: %i", generation_-1);
  ////ROS_INFO("Total number of control cycles:  %i", num_cc_);
  ////ROS_INFO("Exiting Planner::go");
} // End go
