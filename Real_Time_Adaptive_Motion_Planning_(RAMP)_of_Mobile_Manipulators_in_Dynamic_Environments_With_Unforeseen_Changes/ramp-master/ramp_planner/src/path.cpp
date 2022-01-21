#include "path.h"

Path::Path() {}


Path::Path(const KnotPoint start, const KnotPoint goal) : start_(start), goal_(goal) 
{
  msg_.points.push_back(start.buildKnotPointMsg());
  msg_.points.push_back(goal.buildKnotPointMsg());
}

Path::Path(const MotionState start, const MotionState goal) : start_(start), goal_(goal) 
{
  KnotPoint kp_s(start);
  KnotPoint kp_g(goal);

  msg_.points.push_back(kp_s.buildKnotPointMsg());
  msg_.points.push_back(kp_g.buildKnotPointMsg());
}



Path::Path(const std::vector<KnotPoint> all) 
{
  start_ = all.at(0);
  goal_  = all.at(all.size()-1);
  

  for(unsigned int i=0;i<all.size();i++) 
  {
    msg_.points.push_back(all[i].buildKnotPointMsg());
  }
}


Path::Path(const std::vector<MotionState> all) 
{
  for(uint8_t i=0;i<all.size();i++) 
  {
    KnotPoint temp(all[i]);
    msg_.points.push_back(temp.buildKnotPointMsg());
  }
}



Path::Path(const ramp_msgs::Path p) 
{
  //ROS_INFO("In Path::Path");
  //ROS_INFO("p.size(): %i", (int)p.points.size());

  KnotPoint s(p.points.at(0));
  start_ = s;

  KnotPoint g(p.points.at(p.points.size()-1));
  goal_ = g;

  for(unsigned int i=0;i<p.points.size();i++) 
  {
    KnotPoint kp(p.points.at(i));
    msg_.points.push_back(kp.buildKnotPointMsg());
  }
  
  //ROS_INFO("Exiting Path::Path");
}

Path::~Path() {}


const bool Path::equals(const Path& p) const 
{
  //ROS_INFO("In Path::equals");
  
  if(size() != p.size()) {
    //ROS_INFO("Exiting Path::equals");
    return false;
  }

  for(uint8_t i=0;i<size();i++) 
  {
    KnotPoint temp(msg_.points[i]);
    if(!temp.equals(p.msg_.points[i])) 
    {
      //ROS_INFO("Exiting Path::equals");
      return false;
    }
  }

  //ROS_INFO("Exiting Path::equals");
  return true;
}

const KnotPoint Path::at(const uint8_t i) const 
{
  KnotPoint result(msg_.points[i]);
  return result;
}

void Path::offsetPositions(const MotionState diff)
{
  for(uint8_t i=0;i<msg_.points.size()-1;i++)
  {
    MotionState temp(msg_.points[i].motionState);
    temp.subtractPosition(diff);
    msg_.points[i].motionState = temp.msg_;
  }
}

void Path::addBeforeGoal(const KnotPoint kp) 
{
  if(msg_.points.size() > 0) 
  {
    msg_.points.insert(msg_.points.end()-1, kp.buildKnotPointMsg());
  }
  else 
  {
    msg_.points.push_back(kp.buildKnotPointMsg());
  }
}


/** This method inserts the motion state ms into the path at location path_size-1 */
void Path::addBeforeGoal(const MotionState ms) 
{
  KnotPoint kp(ms);
  addBeforeGoal(kp);
}


void Path::changeStart(const MotionState ms) 
{
  KnotPoint kp(ms);
  msg_.points.insert(msg_.points.begin(), kp.buildKnotPointMsg());
  start_ = kp;
}

const unsigned int Path::size() const { return msg_.points.size(); }


const ramp_msgs::Path Path::buildPathMsg() const 
{
  ramp_msgs::Path result;

  //Push all of the configurations onto the Path msg
  for(unsigned int i=0;i<msg_.points.size();i++) 
  {

    //Build the motion state msg
    ramp_msgs::KnotPoint mp = msg_.points[i];

    //Push the msg onto K
    result.points.push_back(mp);
  }

  return result;
}

const std::string Path::toString() const 
{
  std::ostringstream result;

  for(unsigned int i=0;i<msg_.points.size();i++) 
  {
    result<<"\n  "<<i<<": "<<utility_.toString(msg_.points[i]).c_str();
  }
  
  return result.str();
}
