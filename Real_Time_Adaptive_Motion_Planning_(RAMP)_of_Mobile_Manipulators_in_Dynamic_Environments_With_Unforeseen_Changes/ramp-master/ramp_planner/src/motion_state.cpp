#include "motion_state.h"

MotionState::MotionState() : mobile_base_k_(2) {
  msg_.time = -1;
}


MotionState::MotionState(const trajectory_msgs::JointTrajectoryPoint p) : mobile_base_k_(2) {
  for(unsigned int i=0;i<p.positions.size();i++) {
    msg_.positions.push_back(p.positions.at(i));
  }
  
  for(unsigned int i=0;i<p.velocities.size();i++) {
    msg_.velocities.push_back(p.velocities.at(i));
  }
  
  for(unsigned int i=0;i<p.accelerations.size();i++) {
    msg_.accelerations.push_back(p.accelerations.at(i));
  }

  msg_.time = p.time_from_start.toSec();
}

MotionState::MotionState(const ramp_msgs::MotionState ms) : msg_(ms), mobile_base_k_(2) {}




const MotionState MotionState::zero(const uint8_t size) const {
  MotionState result;

  for(uint8_t i=0;i<size;i++) {
    result.msg_.positions.push_back(0);
  }

  for(uint8_t i=0;i<size;i++) {
    result.msg_.velocities.push_back(0);
  }

  for(uint8_t i=0;i<size;i++) {
    result.msg_.accelerations.push_back(0);
  }

  for(uint8_t i=0;i<size;i++) {
    result.msg_.jerks.push_back(0);
  }

  return result;
}

void MotionState::zero()
{

  for(uint8_t i=0;i<msg_.positions.size();i++) {
    msg_.positions[i] = 0;
  }

  for(uint8_t i=0;i<msg_.positions.size();i++) {
    msg_.velocities[i] = 0;
  }

  for(uint8_t i=0;i<msg_.accelerations.size();i++) {
    msg_.accelerations[i] = 0;
  }

  for(uint8_t i=0;i<msg_.jerks.size();i++) {
    msg_.jerks[i] = 0;
  }
}
    
const trajectory_msgs::JointTrajectoryPoint MotionState::getJTP() const
{
  trajectory_msgs::JointTrajectoryPoint result;

  for(uint16_t i=0;i<msg_.positions.size();i++)
  {
    result.positions.push_back(msg_.positions.at(i));
  }

  for(uint16_t i=0;i<msg_.velocities.size();i++)
  {
    result.velocities.push_back(msg_.velocities.at(i));
  }

  for(uint16_t i=0;i<msg_.accelerations.size();i++)
  {
    result.accelerations.push_back(msg_.accelerations.at(i));
  }

  for(uint16_t i=0;i<msg_.jerks.size();i++)
  {
    result.effort.push_back(msg_.jerks.at(i));
  }

  result.time_from_start = ros::Duration(msg_.time);

  return result;
}

/** equals comparison */
const bool MotionState::equals(const MotionState& ms) const 
{
  if(msg_.positions.size() != ms.msg_.positions.size()) 
  {
    return false;
  }
  
  if(msg_.velocities.size() != ms.msg_.velocities.size()) 
  {
    return false;
  }
  
  if(msg_.accelerations.size() != ms.msg_.accelerations.size()) 
  {
    return false;
  }

  if(msg_.jerks.size() != ms.msg_.jerks.size()) 
  {
    return false;
  }

  double epsilon = 0.01;
  for(uint8_t i_p=0;i_p<msg_.positions.size();i_p++) 
  {
    if( fabs(msg_.positions.at(i_p) - ms.msg_.positions.at(i_p)) > epsilon ) 
    {
      return false;
    }
  }

  for(uint8_t i_p=0;i_p<msg_.velocities.size();i_p++) 
  {
    if( fabs(msg_.velocities.at(i_p) - ms.msg_.velocities.at(i_p)) > epsilon ) 
    {
      return false;
    }
  }

  for(uint8_t i_p=0;i_p<msg_.accelerations.size();i_p++) 
  {
    if( fabs(msg_.accelerations.at(i_p) - ms.msg_.accelerations.at(i_p)) > epsilon ) 
    {
      return false;
    }
  }

  for(uint8_t i_p=0;i_p<msg_.jerks.size();i_p++) 
  {
    if( fabs(msg_.jerks.at(i_p) - ms.msg_.jerks.at(i_p)) > epsilon ) 
    {
      return false;
    }
  }

  return true;
}


void MotionState::setEqual(const MotionState ms) {
  msg_ = ms.msg_; 
  mobile_base_k_ = ms.mobile_base_k_;
}



/** 
 * This method returns the euclidean distance between this configuration and c 
 * if base_theta is true, we are considering the base orientation, otherwise do not
 * add base orientation difference into the result
 * */
const double MotionState::comparePosition(const MotionState& c, const bool base_theta) const {
  double result = 0; 

  // For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<msg_.positions.size();i++) {
    // If we are not taking base theta into account, skip i
    if(i == mobile_base_k_ && !base_theta) {}

    // Else if we are considering base theta, use utility_ function
    else if(i == mobile_base_k_) {
      result += pow(utility_.displaceAngle(c.msg_.positions.at(i), msg_.positions.at(i)), 2);
    }
    else {
      result += pow(c.msg_.positions.at(i) - msg_.positions.at(i), 2);
    }
  }

  // Get square root to complete euclidean distance
  result = sqrt(result);

  return result;
}



/** This method returns the new position vector of the Configuration given some transformation matrix */
tf::Vector3 MotionState::transformBasePosition(const tf::Transform t) 
{
  //ROS_INFO("In MotionState::transformBasePosition");
  //ROS_INFO("t: (%f, %f) yaw: %f", t.getOrigin().getX(), t.getOrigin().getY(), tf::getYaw(t.getRotation()));

  //ROS_INFO("msg: %s", toString().c_str());

  tf::Vector3 p(msg_.positions.at(0), msg_.positions.at(1), 0);
  tf::Vector3 result = t * p;

  //ROS_INFO("result: (%f, %f)", result.getX(), result.getY());
  return result;
} //End transformBasePosition



/** This method will transform the configuration by the transformation T
 *  It transforms the position and displaces the orientation by the rotation in T 
 *  The most used source of this method is for updating the robot's configuration */
void MotionState::transformBase(const tf::Transform t) 
{

  // Get the new position
  tf::Vector3 p = transformBasePosition(t);
  msg_.positions.at(0) = p.getX();
  msg_.positions.at(1) = p.getY();
  
  // Get the new orientation
  msg_.positions.at(2) = utility_.displaceAngle(msg_.positions.at(2), tf::getYaw(t.getRotation()));
} //End transformBase





/** */
const MotionState MotionState::add(const MotionState m) const {
  MotionState result = *this;

  for(int i=0;i<msg_.positions.size() && i<m.msg_.positions.size();i++) {
    if(i != mobile_base_k_) {
      result.msg_.positions.at(i) += m.msg_.positions.at(i);
    }
    else {
      result.msg_.positions.at(i) = utility_.displaceAngle(msg_.positions.at(i), m.msg_.positions.at(i));
    }
  }

  /** Separate loops because it's not guaranteed that every MS will have
   * same # of each vector */
  for(int i=0;i<msg_.velocities.size() && i<m.msg_.velocities.size();i++) {
    result.msg_.velocities.at(i) += m.msg_.velocities.at(i);
  }

  for(int i=0;i<msg_.accelerations.size() && i<m.msg_.accelerations.size();i++) {
    result.msg_.accelerations.at(i) += m.msg_.accelerations.at(i);
  }

  for(int i=0;i<msg_.jerks.size() && i<m.msg_.jerks.size();i++) {
    result.msg_.jerks.at(i) += m.msg_.jerks.at(i);
  }

  return result;
} // End add




/** */
const MotionState MotionState::subtractPosition(const MotionState m, bool orientation) const {
  //ROS_INFO("In MotionState::subtract");

  MotionState result = *this;

  for(int i=0;i<msg_.positions.size() && i<m.msg_.positions.size();i++) {
    if(i == mobile_base_k_ && orientation)
    {
      result.msg_.positions.at(i) = utility_.displaceAngle(result.msg_.positions.at(i), -m.msg_.positions.at(i));
    }
    else 
    {
      result.msg_.positions.at(i) -= m.msg_.positions.at(i);
    }
    /*else {
      result.msg_.positions.at(i) = utility_.displaceAngle(msg_.positions.at(i), -m.msg_.positions.at(i));
    }*/
  }


  //ROS_INFO("Exiting MotionState::subtract");
  return result;
} // End subtract



/** Probably not going to stay here 
 * Used for dividing delta_m in the planner code
 * may need to use displaceAngle for theta position */
const MotionState MotionState::multiply(const int num) const {
  MotionState result = *this;

  for(int i=0;i<msg_.positions.size();i++) {
    result.msg_.positions.at(i) *= num;
  }
  
  for(int i=0;i<msg_.velocities.size();i++) {
    result.msg_.velocities.at(i) *= num;
  }

  for(int i=0;i<msg_.accelerations.size();i++) {
    result.msg_.accelerations.at(i) *= num;
  }

  for(int i=0;i<msg_.jerks.size();i++) {
    result.msg_.jerks.at(i) *= num;
  }

  return result;
}




/** Probably not going to stay here 
 * Used for dividing delta_m in the planner code */
const MotionState MotionState::divide(const int num) const {
  MotionState result = *this;

  for(int i=0;i<msg_.positions.size();i++) {
      result.msg_.positions.at(i) /= num;
  }
  
  for(int i=0;i<msg_.velocities.size();i++) {
      result.msg_.velocities.at(i) /= num;
  }

  for(int i=0;i<msg_.accelerations.size();i++) {
      result.msg_.accelerations.at(i) /= num;
  }

  for(int i=0;i<msg_.jerks.size();i++) {
      result.msg_.jerks.at(i) /= num;
  }

  return result;
}



const MotionState MotionState::abs() const {
  MotionState result = *this;

  for(unsigned int i=0;i<msg_.positions.size();i++) {
    result.msg_.positions.at(i) = fabs(result.msg_.positions.at(i));
  }

  for(unsigned int i=0;i<msg_.velocities.size();i++) {
    result.msg_.velocities.at(i) = fabs(result.msg_.velocities.at(i));
  }

  for(unsigned int i=0;i<msg_.accelerations.size();i++) {
    result.msg_.accelerations.at(i) = fabs(result.msg_.accelerations.at(i));
  }
  
  for(unsigned int i=0;i<msg_.jerks.size();i++) {
    result.msg_.jerks.at(i) = fabs(result.msg_.jerks.at(i));
  }

  return result;
}



const double MotionState::normPosition() const {
  double result = 0;

  for(uint16_t i=0;i<msg_.positions.size();i++) {
    result += pow(msg_.positions.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::normVelocity() const {
  double result = 0;

  for(uint16_t i=0;i<msg_.velocities.size();i++) {
    result += pow(msg_.velocities.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::normAcceleration() const {
  double result = 0;

  for(uint16_t i=0;i<msg_.accelerations.size();i++) {
    result += pow(msg_.accelerations.at(i), 2);
  }

  result = sqrt(result);

  return result;
}



const double MotionState::normJerk() const {
  double result = 0;

  for(uint16_t i=0;i<msg_.jerks.size();i++) {
    result += pow(msg_.jerks.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::norm() const {
  double result = 0;

  result += normPosition();
  result += normVelocity();
  result += normAcceleration();
  result += normJerk();

  result = sqrt(result);

  return result;
}




const std::string MotionState::toString() const {
  std::ostringstream result;
  
  //msg_.positions
  if(msg_.positions.size() == 0) {
    result<<"\np: []";
  }
  else {
    result<<"\np: ["<<msg_.positions.at(0);
    for(unsigned int i=1;i<msg_.positions.size();i++) {
      result<<", "<<msg_.positions.at(i);
    }
    result<<"]";
  }

  //msg_.velocities
  if(msg_.velocities.size() == 0) {
    result<<"\nv: []";
  }
  else {
    result<<"\nv: ["<<msg_.velocities.at(0);
    for(unsigned int i=1;i<msg_.velocities.size();i++) {
      result<<", "<<msg_.velocities.at(i);
    }
    result<<"]";
  }

  //msg_.accelerations
  if(msg_.accelerations.size() > 0) {
    result<<"\na: ["<<msg_.accelerations.at(0);
    for(unsigned int i=1;i<msg_.accelerations.size();i++) {
      result<<", "<<msg_.accelerations.at(i);
    }
    result<<"]";
  }

  // msg_.jerks
  if(msg_.jerks.size() > 0) {
    result<<"\nj: ["<<msg_.jerks.at(0);
    for(unsigned int i=1;i<msg_.jerks.size();i++) {
      result<<", "<<msg_.jerks.at(i);
    }
    result<<"]";
  }

  result<<"\nTime from start: "<<msg_.time;
  result<<"\n----\n";

  return result.str();
}
