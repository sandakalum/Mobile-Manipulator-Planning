#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(unsigned int id) 
{
  msg_.id = id;
  msg_.feasible = true;
  msg_.fitness = -1;  
  msg_.t_firstCollision = ros::Duration(9999.f);
  msg_.t_start          = ros::Duration(0.f);
  msg_.trajectory.points.reserve(100);
}

RampTrajectory::RampTrajectory(const ramp_msgs::RampTrajectory msg) : msg_(msg) {}


const bool RampTrajectory::equals(const RampTrajectory& other) const 
{
  ////ROS_INFO("In RampTrajectory::equals");
  if(msg_.id == other.msg_.id) 
  {
    return true;
  }

  Path templ(msg_.holonomic_path);
  Path tempr(other.msg_.holonomic_path);

  ////ROS_INFO("Exiting RampTrajectory::equals");
  //return msg_.holonomic_path.equals(other.msg_.holonomic_path);
  return templ.equals(tempr);
}


const double RampTrajectory::getT() const
{
  return msg_.trajectory.points.at(msg_.trajectory.points.size()-1).time_from_start.toSec();
}

const Path RampTrajectory::getNonHolonomicPath() const 
{
  Path result;

  for(unsigned int i=0;i<msg_.i_knotPoints.size();i++) 
  {

    MotionState ms(msg_.trajectory.points.at( msg_.i_knotPoints.at(i)));
    KnotPoint kp_ms(ms);

    result.msg_.points.push_back(kp_ms.buildKnotPointMsg());
  }

  result.start_ = result.msg_.points[0];
  result.goal_  = result.msg_.points[ result.msg_.points.size()-1 ];
  
  return result;
}


/** Time is in seconds */
const trajectory_msgs::JointTrajectoryPoint RampTrajectory::getPointAtTime(const float t) const 
{
  //////ROS_INFO("In RampTrajectory::getPointAtTime");
  
  
  float resolutionRate = 0.1;
  int i = ceil((t/resolutionRate));
  /*////ROS_INFO("t: %f resolutionRate: %f i: %i size: %i", 
      t, 
      resolutionRate, 
      i, 
      (int)msg_.trajectory.points.size());*/

  if( i >= msg_.trajectory.points.size() ) {
    return msg_.trajectory.points.at( msg_.trajectory.points.size()-1 );
  }

  return msg_.trajectory.points.at(i);
}





/** Returns the direction of the trajectory, i.e. the
* orientation the base needs to move on the trajectory */
const double RampTrajectory::getDirection() const 
{
  //std::cout<<"\nIn getDirection\n";
  std::vector<double> a = msg_.holonomic_path.points[0].motionState.positions;

  std::vector<double> b = msg_.holonomic_path.points[1].motionState.positions;

    //msg_.trajectory.points.at(msg_.i_knotPoints.at(2)) :
    //msg_.trajectory.points.at(msg_.i_knotPoints.at(1)) ;
  //std::cout<<"\nLeaving getDirection\n";
  return utility_.findAngleFromAToB(a, b);
}




// Inclusive
// TODO: Change for loop to only use integers because it's a pain to deal with floating point +,-
const RampTrajectory RampTrajectory::getSubTrajectory(const float t) const 
{
  //////ROS_INFO("In RampTrajectory::getSubTrajectory");
  //////ROS_INFO("t: %f size: %i", t, (int)msg_.trajectory.points.size());
  ramp_msgs::RampTrajectory rt;

  double t_stop = t;

  if(msg_.trajectory.points.size() < 1)
  {
    ////ROS_ERROR("msg_.trajectory.points.size == 0");
    return clone();
  }
  else if(msg_.trajectory.points.size() == 1)
  {
    rt.trajectory.points.push_back(msg_.trajectory.points.at(0));
    rt.i_knotPoints.push_back(0);
  }

  else
  {
    if( t > msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec()) 
    {
      t_stop = msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec();
    }


    uint8_t i_kp = 0;
    for(float i=0.f;i<=t_stop+0.0001;i+=0.100001f) 
    { 
      uint16_t index = floor(i*10.) < msg_.trajectory.points.size() ? floor(i*10) : 
        msg_.trajectory.points.size()-1;

      //////ROS_INFO("index: %i size: %i i_kp: %i msg_.i_knotPoints.size(): %i", index, (int)msg_.trajectory.points.size(), 
          //i_kp, (int)msg_.i_knotPoints.size());
      rt.trajectory.points.push_back(msg_.trajectory.points.at(index)); 
      if(i_kp < msg_.i_knotPoints.size() && msg_.i_knotPoints.at(i_kp) == index) 
      {
        rt.i_knotPoints.push_back(index);
        i_kp++;
      } // end if
    } // end for
    
    // If the last point was not a knot point, make it one for the sub-trajectory
    if(rt.i_knotPoints.at( rt.i_knotPoints.size()-1 ) != rt.trajectory.points.size()-1)
    {
      rt.i_knotPoints.push_back(rt.trajectory.points.size()-1);
    }
  }

  RampTrajectory result(rt);

  return result;
}


const RampTrajectory RampTrajectory::getSubTrajectoryPost(const double t) const
{
  //////ROS_INFO("In RampTrajectory::getSubTrajectoryPost");
  RampTrajectory rt;

  double t_start = t;

  if(msg_.trajectory.points.size() < 1)
  {
    //ROS_ERROR("msg_.trajectory.points.size == 0");
    return clone();
  }
  else if(msg_.trajectory.points.size() == 1 ||
      t > msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec())
  {
    rt.msg_.trajectory.points.push_back(msg_.trajectory.points.at(0));
    rt.msg_.i_knotPoints.push_back(0);
  }
  
  
  else
  {
    rt.msg_.i_knotPoints.push_back(0);
    uint8_t i_kp = 1;
    
    // Push on all the points
    //for(float i=t_start;i<=t_stop;i+=0.1f) 
    for(int i=floor(t_start*10.);i<msg_.trajectory.points.size();i++) 
    {

      // Get point
      trajectory_msgs::JointTrajectoryPoint p = msg_.trajectory.points.at(i);
      
      // Adjust time
      p.time_from_start = ros::Duration(p.time_from_start.toSec() - t_start);

      // Add to trajectory
      rt.msg_.trajectory.points.push_back(p); 

      // If it's a knot point, push on it's index
      if(msg_.i_knotPoints.at(i_kp) == i) 
      {
        rt.msg_.i_knotPoints.push_back(rt.msg_.trajectory.points.size()-1);
        i_kp++;
      }
    } // end for


    // If the last point was not a knot point, make it one for the sub-trajectory
    if(rt.msg_.i_knotPoints.at( rt.msg_.i_knotPoints.size()-1 ) != rt.msg_.trajectory.points.size()-1)
    {
      rt.msg_.i_knotPoints.push_back(rt.msg_.trajectory.points.size()-1);
    }

    // Make a path
    /*Path temp(rt.msg_.trajectory.points.at(0), rt.msg_.trajectory.points.at(rt.msg_.trajectory.points.size()-1));
    for(uint8_t i=0;i<path_.size();i++)
    {
      if(path_.at(i).motionState_.msg_.time > t_start)
      {
        temp.addBeforeGoal(path_.at(i));
      }
    }

    if(msg_.i_curveEnd >= (t*10.))
    {
      KnotPoint k(msg_.curves.at(0).segmentPoints.at(0));
      temp.all_.insert(temp.all_.begin()+1, k);
    }

    rt.path_ = temp;*/
  } // end else

  //////ROS_INFO("Returning sub-trajectory: %s", rt.toString().c_str());
  //////ROS_INFO("Exiting RampTrajectory::getSubTrajectoryPost");
  return rt;
}




/*
 * Concatenate traj onto this trajectory. kp is the knot to start on traj
 */
const RampTrajectory RampTrajectory::concatenate(const RampTrajectory traj, const uint8_t kp) const 
{
  //////ROS_INFO("In RampTrajectory::concatenate");
  //////ROS_INFO("traj: %s", traj.toString().c_str());
  //////ROS_INFO("kp: %i", kp);
  
  RampTrajectory result = clone();
  uint8_t c_kp = kp+1;

  if(msg_.trajectory.points.size() == 0)
  {
    ////ROS_WARN("msg_trajectory.points.size() == 0, Returning parameter traj");
    return traj;
  }

  if(traj.msg_.trajectory.points.size() == 0)
  {
    ////ROS_WARN("traj.msg_trajectory.points.size() == 0, Returning *this");
    return *this;
  }
  
  if(traj.msg_.i_knotPoints.size() == 1)
  {
    ////ROS_WARN("traj.msg_.i_knotPoints.size() <= kp: %i, returning *this", (int)kp);
    return *this;
  }

  if(traj.msg_.i_knotPoints.size() <= kp+1)
  {
    ////ROS_WARN("traj.msg_.i_knotPoints.size() <= kp: %i, returning *this", (int)kp);
    return *this;
  }
  


  //////ROS_INFO("traj.msg_.trajectory.points.size(): %i i_knotpoints.at(%i): %i", (int)traj.msg_.trajectory.points.size(), kp, traj.msg_.i_knotPoints.at(kp));

  /*
   * Test that the last point in this trajectory matches the first point in traj
   */
  trajectory_msgs::JointTrajectoryPoint last  = msg_.trajectory.points.at(
                                                msg_.trajectory.points.size()-1);
  trajectory_msgs::JointTrajectoryPoint first = traj.msg_.trajectory.points.at(traj.msg_.i_knotPoints.at( kp ));
  if( fabs(utility_.positionDistance(last.positions, first.positions)) > 0.1)
  {
    //////ROS_WARN("First and last points don't match!");
    //////ROS_WARN("last: %s\nfirst: %s\ndiff: %f", utility_.toString(last).c_str(), utility_.toString(first).c_str(), fabs(utility_.positionDistance(last.positions, first.positions)));
    return *this;
  }

  /*////ROS_INFO("traj.msg_.trajectory.points.size(): %i", (int)traj.msg_.trajectory.points.size());
  ////ROS_INFO("kp: %i", kp);
  ////ROS_INFO("traj.msg_.i_knotPoints.size(): %i", (int)traj.msg_.i_knotPoints.size());
  ////ROS_INFO("traj.msg_.i_knotPoints.at(%i): %i", kp+1, traj.msg_.i_knotPoints.at(kp+1));*/
  
  trajectory_msgs::JointTrajectoryPoint endOfFirstSegment = traj.msg_.trajectory.points.at(traj.msg_.i_knotPoints.at( kp+1 ));

  // If there is no curve AND
  // If there is only two knot points AND
  // If the last segment of this and first segment of traj have the same orientation
  // Remove the last knot point of this trajectory
  // E.g. trajectories (0,0)->(1,0) and (1,0)->(2,0) concatenate to (0,0)->(2,0)
  // *** Assumes that first segment of target does not start a curve ***
  if( msg_.curves.size() == 0 && 
      msg_.i_knotPoints.size() < 3 &&
      traj.msg_.curves.size() == 0 &&
      utility_.findDistanceBetweenAngles(last.positions.at(2), endOfFirstSegment.positions.at(2)) < 0.01)
  {
    /*////ROS_INFO("Last segment of this and first segment of traj have the same orientation");
    ////ROS_INFO("last.positions.at(2): %f first.positions.at(2): %f", 
        last.positions.at(2), first.positions.at(2));
    ////ROS_INFO("Removing last knotpoint of this trajectory");*/
    result.msg_.i_knotPoints.pop_back();
  }


  // Find the new size of the trajectory
  int initial_size  = result.msg_.trajectory.points.size();
  int traj_size     = traj.msg_.i_knotPoints.size() == 1 ? 0 :
                      traj.msg_.trajectory.points.size() - traj.msg_.i_knotPoints.at(c_kp-1);
  int total_size = initial_size + traj_size;

  // Reserve space
  result.msg_.trajectory.points.reserve(total_size);
  
  // Get time variables
  ros::Duration t_cycleTime(0.1);
  ros::Duration t_latest =  msg_.trajectory.points.at
                            (initial_size-1).time_from_start;

 // Push on the points
  for(uint16_t  i=traj.msg_.i_knotPoints.size() == 1 ? 0 : traj.msg_.i_knotPoints.at(c_kp-1)+1 ;
    i<traj.msg_.trajectory.points.size() ;
    i++)
  {
   // Get the point and adjust the time
    trajectory_msgs::JointTrajectoryPoint temp  = traj.msg_.trajectory.points.at(i);
    temp.time_from_start                        = ros::Duration(t_latest+t_cycleTime);
    result.msg_.trajectory.points.push_back(temp);

    // If it's a knot point, push back the index
    if( i == traj.msg_.i_knotPoints.at(c_kp) )
    {
      //////ROS_INFO("i: %i traj.msg_.i_knotPoints.at(%i): %i", i, c_kp, traj.msg_.i_knotPoints.at(c_kp));
      //////ROS_INFO("temp: %s", utility_.toString(temp).c_str());
      result.msg_.i_knotPoints.push_back( result.msg_.trajectory.points.size()-1 ); 
      c_kp++;
    }
    
    t_latest += t_cycleTime;
  } //end for

  // Push on the target trajectory's Bezier curve
  for(uint8_t i_curve=0;i_curve<traj.msg_.curves.size();i_curve++) 
  {
    //////ROS_INFO("Pushing on curve %i", i_curve);
    result.msg_.curves.push_back(traj.msg_.curves.at(i_curve));
  }


  //////ROS_INFO("result: %s", result.toString().c_str());
  //////ROS_INFO("Exiting RampTrajectory::concatenate");
  return result;
}



/*
 * path_ is mutated in this method! The knot points of the Path are also offset by diff
 * The BezierCurves have their segment and control points offset
 */
void RampTrajectory::offsetPositions(const MotionState& diff)
{
  //////ROS_INFO("In RampTrajectory::offsetPositions");
 
  // Go through all the points and subtract diff
  for(uint16_t i=0;i<msg_.trajectory.points.size();i++)
  {
    MotionState temp(msg_.trajectory.points.at(i));
    temp = temp.add(diff);

    // Set new positions
    for(uint8_t j=0;j<msg_.trajectory.points.at(i).positions.size();j++)
    {
      msg_.trajectory.points.at(i).positions.at(j) = temp.msg_.positions.at(j);
    }
  } // end outter for
  //////ROS_INFO("Done offsetting points");

  if(msg_.holonomic_path.points.size() > 0)
  {
    Path temp(msg_.holonomic_path);
    temp.offsetPositions(diff);
    msg_.holonomic_path = temp.msg_;
    //msg_.holonomic_path.offsetPositions(diff);
  }
  else
  {
    //////ROS_WARN("path_.size() == 0, not touching it");
  }


  /*if(msg_.curves.size() > 0 && path_.size() < 3)
  {
    ////ROS_WARN("temp.msg_.curves.size() > 0 && temp.path_.size() < 3");
    ////ROS_WARN("temp.path_: %s", path_.toString().c_str());
    ////ROS_WARN("temp.curve.at(0): %s", utility_.toString(msg_.curves.at(0)).c_str());
  }*/

  for(uint8_t c=0;c<msg_.curves.size() && msg_.holonomic_path.points.size() > 2;c++)
  {
    /*////ROS_INFO("Fixing curve %i, msg_.holonomic_path..size(): %i segmentPoints.size(): %i controlPoints.size(): %i", 
        c, 
        msg_.holonomic_path..size(), 
        (int)msg_.curves.at(c).segmentPoints.size(), 
        (int)msg_.curves.at(c).controlPoints.size());*/

    msg_.curves.at(c).segmentPoints.at(1) = msg_.holonomic_path.points.at(1).motionState;
    msg_.curves.at(c).segmentPoints.at(2) = msg_.holonomic_path.points.at(2).motionState;

    MotionState c0(msg_.curves.at(c).controlPoints.at(0));
    MotionState c1(msg_.curves.at(c).controlPoints.at(1));
    MotionState c2(msg_.curves.at(c).controlPoints.at(2));

    msg_.curves.at(c).controlPoints.at(0) = c0.subtractPosition(diff).msg_;
    msg_.curves.at(c).controlPoints.at(1) = c1.subtractPosition(diff).msg_;
    msg_.curves.at(c).controlPoints.at(2) = c2.subtractPosition(diff).msg_;
    //////ROS_INFO("After fixing curve: %s", utility_.toString(msg_.curves.at(0)).c_str());
  } // end for


  //////ROS_INFO("Exiting RampTrajectory::offsetPositions");
} // End offsetPositions



const RampTrajectory RampTrajectory::clone() const 
{ 
  return *this;
}


const std::string RampTrajectory::fitnessFeasibleToString() const 
{
  std::ostringstream result;
 
  result<<"\nTrajectory ID: "<<msg_.id;
  result<<"\n Holonomic Path: "<<utility_.toString(msg_.holonomic_path).c_str();
  result<<"\n Non-holonomic Path: "<<getNonHolonomicPath().toString();
  result<<"\n t_start: "<<msg_.t_start.toSec()<<" Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;

  return result.str();
}

const std::string RampTrajectory::toString() const 
{
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\nTrajec: "<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Collision Time: "<<msg_.t_firstCollision<<" Feasible: "<<(int)msg_.feasible;
  result<<"\n   Transition traj size: "<<transitionTraj_.trajectory.points.size();
  
  return result.str();
}




