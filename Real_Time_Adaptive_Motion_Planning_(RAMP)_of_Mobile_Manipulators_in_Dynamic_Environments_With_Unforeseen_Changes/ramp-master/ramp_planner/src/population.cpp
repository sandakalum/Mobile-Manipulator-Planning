#include "population.h"

Population::Population() : type_(HYBRID), maxSize_(3), isSubPopulation_(false) {}

Population::Population(const unsigned int size, const TrajectoryType type, const bool isSubPop) : type_(type), maxSize_(size), isSubPopulation_(isSubPop) {}


/** Return the size of the population */
const unsigned int Population::size() const { return trajectories_.size(); }

void Population::clear() 
{ 
  trajectories_.clear(); 
  paths_.clear();

  if(subPopulations_.size() > 0)
  {
    subPopulations_.clear();
  }
}


const std::vector<RampTrajectory> Population::getTrajectories() const {return trajectories_;}

/** This method returns the trajectory at index i */
const RampTrajectory Population::get(const unsigned int i) const 
{
  return trajectories_.at(i);
} // End get



/** Returns -1 if it cannot find the id */
const int Population::getIndexFromId(const uint16_t id) const 
{
  for(uint16_t i=0;i<trajectories_.size();i++) 
  {
    if(trajectories_.at(i).msg_.id == id)
    {
      return i;
    }
  }

  return -1;
}



void Population::replace(const uint8_t i, const RampTrajectory& trajec) 
{
  if(i < trajectories_.size()) 
  {
    trajectories_.at(i) = trajec;
    paths_.at(i) = trajec.msg_.holonomic_path;
  }
  else 
  {
    //ROS_WARN("Replacing trajectory at index %i, but population size = %lu\n", (int)i, trajectories_.size());
    trajectories_.push_back(trajec);
    paths_.push_back(trajec.msg_.holonomic_path);  
  }
}



void Population::replaceAll(const std::vector<RampTrajectory>& new_pop) 
{
  
  // Set trajectories vector
  trajectories_ = new_pop;

  // Set paths vector
  paths_.clear();
  for(uint8_t i=0;i<new_pop.size();i++) 
  {
    paths_.push_back(new_pop.at(i).msg_.holonomic_path);
  }
} // End replaceAll





const int Population::getNumSubPops() const 
{
  return subPopulations_.size();
}





/** This method returns the minimum fitness of the population */
const double Population::getMinFitness() const {
  double result = trajectories_.at(0).msg_.fitness;

  for(uint8_t i=1;i<trajectories_.size();i++) {
    if(trajectories_.at(i).msg_.fitness < result) {
      result = trajectories_.at(i).msg_.fitness;
    }
  }
  
  return result;
} // End getMinFitness




const bool Population::contains(const RampTrajectory& rt) const 
{
  //ROS_INFO("In Population::contains");
  for(uint8_t i=0;i<size();i++) 
  {
    //ROS_INFO("i: %i size(): %i", i, size());
    if(trajectories_.at(i).equals(rt)) 
    {
      //ROS_INFO("Population contains trajectory at index %i", (int)i);
      return true;
    }
  }

  //ROS_INFO("Population does not contain trajectory");
  return false;
}






/** This method returns true if there is at least one feasible trajectory in the population */
const bool Population::feasibleExists() const 
{
  for(uint8_t i=0;i<trajectories_.size();i++) 
  {
    if(trajectories_.at(i).msg_.feasible) 
    {
      return true;
    }
  }

  return false;
} // End feasibleExists



/** This method returns true if there is at least one infeasible trajectory in the population */
const bool Population::infeasibleExists() const 
{
  for(uint8_t i=0;i<trajectories_.size();i++) 
  {
    if(!trajectories_.at(i).msg_.feasible) 
    {
      return true;
    }
  }

  return false;
} // End infeasibleExists








/** This method checks if a trajectory can replace an existing trajectory in the population */
const bool Population::replacementPossible(const RampTrajectory& rt) const 
{
  //ROS_INFO("In Population::replacementPossible");
  //////ROS_INFO("rt: %s", rt.toString().c_str());
  //ROS_INFO("rt.fitness: %f feasible: %s t_coll: %f", rt.msg_.fitness, rt.msg_.feasible ? "True" : "False", rt.msg_.t_firstCollision.toSec());
  //////ROS_INFO("pop: %s", toString().c_str());
  
  /*////ROS_INFO("Trajec 1 fit: %f feasible: %s t_coll: %f", trajectories_.at(1).msg_.fitness, 
      trajectories_.at(1).msg_.feasible ? "True" : "False", trajectories_.at(1).msg_.t_firstCollision);*/

  //ROS_INFO("getMinFitness(): %f", getMinFitness());

  // If the fitness is not higher than the minimum fitness
  /*if(rt.msg_.fitness <= getMinFitness()) 
  {
    ROS_INFO("Fitness < Minimum Fitness, %f < %f, returning false", rt.msg_.fitness, getMinFitness());
    return false;
  }*/
  
  // If the trajectory is infeasible and
  // no infeasible trajectories exist, no
  // trajectories can be replaced
  if(!rt.msg_.feasible && !infeasibleExists()) 
  {
    //ROS_INFO("rt is infeasible, no other infeasible trajecs, returning false");
    return false;
  }

  /** IF subpopulations are being used */
  if(subPopulations_.size() > 0) 
  {
    //std::cout<<"\nIn sub-pops\n";
    
    // If each subpopulation has <= 1 trajectory,
    // no trajectories can be replaced
    std::vector<uint8_t> i_validSubpops;
    for(uint8_t i=0;i<subPopulations_.size();i++) 
    {
      if(subPopulations_.at(i).size() > 1 && rt.msg_.fitness > subPopulations_.at(i).getMinFitness()) 
      {
        i_validSubpops.push_back(i);
      }
    }
    if(i_validSubpops.size() == 0) 
    {
      //std::cout<<"\nAll sub-pops size < 2, not possible\n";
      return false;
    }
    else 
    {
      //std::cout<<"\ni_validSubPops.size(): "<<i_validSubpops.size()<<"\n";
    }
  
    // If the valid sub-populations have only feasible trajectories
    // no trajectories can be replaced
    // or if rt's fitness is lower than the min fitness of sub-population
    if(!rt.msg_.feasible) 
    {
      ////ROS_INFO("In !feasible");
      ////ROS_INFO("i_validSubpops.size: %i", (int)i_validSubpops.size());
      bool valid=false;
      for(uint8_t i=0;i<i_validSubpops.size();i++) 
      {
        ////ROS_INFO("i: %i", i);
        if(subPopulations_.at(i_validSubpops.at(i)).infeasibleExists() &&
            rt.msg_.fitness > subPopulations_.at(i_validSubpops.at(i)).getMinFitness())
        {
          valid = true;
        }
      }

      if(!valid) 
      {
        ////ROS_INFO("Not valid, returning false");
        return false;
      }
    } // end if rt is infeasible
  } // end if sub-populations are used

  //ROS_INFO("Replacement is possible");
  //ROS_INFO("Exiting Population::replacementPossible");
  return true;
} // End replacementPossible






/** This method returns true if rt can replace the trajectory at index i */
const bool Population::canReplace(const RampTrajectory& rt, const int& i) const 
{
  //ROS_INFO("In Population::canReplace");
  //ROS_INFO("i: %i feasible: %s", i, trajectories_.at(i).msg_.feasible ? "True" : "False");
  
  if(i == calcBestIndex()) 
  {
    //ROS_INFO("i == i_best, returning false");
    return false;
  }

  if(!rt.msg_.feasible && trajectories_.at(i).msg_.feasible) 
  {
    //ROS_INFO("rt infeasible, i feasible, returning false");
    return false;
  }


  // If sub-populations are used,
  if(subPopulations_.size() > 0) 
  {
    //std::cout<<"\nIn sub-pops are being used!";
    
    //std::cout<<"\ntrajectories.size(): "<<trajectories_.size();
    RampTrajectory temp = trajectories_.at(i);

    //std::cout<<"\nsubPopulations.size(): "<<subPopulations_.size();
    //std::cout<<"\ntemp.i_subPopulation: "<<temp.msg_.i_subPopulation<<"\n";
    Population p = subPopulations_.at(temp.msg_.i_subPopulation);

    if(p.trajectories_.size() < 2) 
    {
      //std::cout<<"\nSub-Population size < 2, returning false\n";
      return false;
    }

    //std::cout<<"\np.trajectories.size(): "<<p.trajectories_.size();
    //std::cout<<"\np.calcBestIndex(): "<<p.calcBestIndex()<<"\n";

    // if i is the best in trajectory i's sub-population
    if(temp.equals( p.trajectories_.at(p.calcBestIndex()) )) 
    {
      //std::cout<<"\ntemp == best in sub-population, returning false\n";
      return false;
    }

    if(rt.msg_.fitness < temp.msg_.fitness)
    {
      return false;
    }
  } // end if sub-pops are being used
  
  //ROS_INFO("Returning true");
  //ROS_INFO("Exiting Population::canReplace");
  return true;
} // End canReplace







/** This method determines which trajectory (if any) in the
* population will be replaced if the population is full when
* adding a trajectory to it
* A result of -1 means no trajectories could be removed
* - Will happen if rt is infeasible and the rest are feasible
* - or if each sub-population has <= 1 trajectory */
const int Population::getReplacementID(const RampTrajectory& rt) const 
{
  //ROS_INFO("In Population::getReplacementID");
  //std::cout<<"\nIn getReplacementID\n";
  //std::cout<<"\n"<<toString();
  
  // If the trajectory is infeasible and
  // no infeasible trajectories exist, no
  // trajectories can be replaced
  int result;
  
  // Generate a random index for a random trajectory to remove
  do 
  {
    result = rand() % trajectories_.size();
    ////ROS_INFO("result: %i", result); 
  }
  
  // Keep getting a random index until it
  // is an index that rt can replace
  while(!canReplace(rt, result));

  
  //ROS_INFO("Exiting Population::getReplacementID");
  return result;
} // End getReplacementID






/** This method adds a trajectory to the population. 
 *  If the population is full, a random trajectory (that isn't the best one) is replaced
 *  Returns the index that the trajectory is added at */
const int Population::add(const RampTrajectory& rt) 
{
  /*ROS_INFO("In Population::add");
  ROS_INFO("Pop: %s", toString().c_str());
  ROS_INFO("rt: %s", rt.toString().c_str());
  ROS_INFO("Pop best id: %i", calcBestIndex());*/

  /*if(subPopulations_.size() > 0) 
  {
    // Go through each sub-population and find best
    for(uint8_t i=0;i<subPopulations_.size();i++) 
    {
      subPopulations_.at(i).calcBestIndex();
    }
  }*/
 
  // If it's a sub-population or
  // If it's not full, simply push back
  if(isSubPopulation_ || trajectories_.size() < maxSize_) 
  {
    trajectories_.push_back (rt);  
    paths_.push_back        (rt.msg_.holonomic_path);
    
    //ROS_INFO("In if isSubPopulation_ || trajectories_.size() < maxSize_");
    ////ROS_INFO("Exiting Population::add");
    return trajectories_.size()-1;
  }

  // If full, replace a trajectory
  else if(!contains(rt) && replacementPossible(rt)) 
  {
    int i = getReplacementID(rt);

    replace(i, rt);
    
    //ROS_INFO("Added trajectory to index %i", i);
    ////ROS_INFO("Exiting Population::add");
    return i;
  }


  /*//ROS_INFO("Cannot add trajectory");
  //ROS_INFO("%s", rt.toString().c_str());
  //ROS_INFO("pop: %s", toString().c_str());*/

  ////ROS_INFO("Exiting Population::add");
  return -1;
} //End add



/** Returns the fittest trajectory and sets calcBestIndex() */
const int Population::calcBestIndex() const 
{
  if(size() == 0)
  {
    ROS_ERROR("Calling Population::calcBestIndex(), but Population is empty");
    //ROS_INFO("Pop: %s", toString().c_str());
    return -1;
  }
 
  // Find the index of the trajectory with the highest fitness value
  int i_max = 0;
  for(int i=1;i<trajectories_.size();i++) 
  {
    if(trajectories_.at(i).msg_.fitness > trajectories_.at(i_max).msg_.fitness) 
    {
      i_max = i;
    }
  } //end for


  return i_max; 
} //End calcBestIndex



const RampTrajectory Population::getBest() const 
{
  if(size() == 0)
  {
    //ROS_ERROR("Calling Population::getBest(), but Population is empty");
  }
  return trajectories_.at(calcBestIndex());
}


void Population::setStartTime(double t_start) 
{
  for(int i=0;i<trajectories_.size();i++)
  {
    trajectories_[i].msg_.t_start = ros::Duration(t_start);
  }
}

const ros::Duration Population::getEarliestStartTime() const
{
  ros::Duration result = trajectories_.at(0).msg_.t_start;

  // Get min t_start
  for(uint8_t i=1;i<trajectories_.size();i++)
  {
    if(trajectories_.at(i).msg_.t_start < result)
    {
      result = trajectories_.at(i).msg_.t_start;
    }
  } 

  return result;
}



/** This method will return the best trajectory from each sub-population */
const std::vector<RampTrajectory> Population::getBestFromSubPops() const
{
  std::vector<RampTrajectory> result;

  if(subPopulations_.size() == 0) 
  {
    //ROS_ERROR("Calling Population::getBestFromSubPops, but Population has no sub-populations!");
  }

  else {
    for(uint8_t i=0;i<subPopulations_.size();i++) {
      if(subPopulations_.at(i).size() > 0) {
        int i_best = subPopulations_.at(i).calcBestIndex(); 
        result.push_back(subPopulations_.at(i).get(i_best));
      }
    }
  }

  return result;
}



/** This method creates sub-populations based on delta_theta.
* It will set the subPopulations_ member and also return the sub populations */
const std::vector<Population> Population::createSubPopulations(const double delta_theta) 
{
  subPopulations_.clear();


  // Get the number of sub-pops for delta_theta
  int num = ceil((2*PI) / delta_theta);
  ////ROS_INFO("num: %i", num);
 
  // Create the sub-populations
  for(uint8_t i=0;i<num;i++) 
  {
    Population sub(maxSize_, type_, true);
    subPopulations_.push_back(sub);
  }

  // Go through each trajectory
  for(uint8_t i=0;i<trajectories_.size();i++) 
  {

    // Get direction and Convert to [0,2PI]
    double departure_direction = trajectories_.at(i).getDirection();
    if(departure_direction < 0)
    {
      departure_direction += (2*PI);
    }
    

    // Find the sub-pop it belongs to
    // and add it to that sub-pop
    for(uint8_t sp=0;sp<num;sp++) 
    {
      if(departure_direction < delta_theta*(sp+1)) 
      {
        subPopulations_.at(sp).add(trajectories_.at(i));
        trajectories_.at(i).msg_.i_subPopulation = sp;
        sp = num;
      }
    } // end inner loop
  } // end outer loop


  // Go through each sub-population and find best
  for(uint8_t i=0;i<subPopulations_.size();i++) 
  {
    subPopulations_.at(i).calcBestIndex();
  }

  //std::cout<<"\n***********Leaving createSubPopulations***********\n";
  return subPopulations_;
}





/** fitness and feasible toString */
const std::string Population::fitnessFeasibleToString() const 
{
  std::ostringstream result;

  result<<"\n****************************************************";
  result<<"\nPopulation's fitness and feasibility:";
  for(unsigned int i=0;i<trajectories_.size();i++) {
    result<<"\n"<<trajectories_.at(i).fitnessFeasibleToString();
    //if(i == calcBestIndex()) 
    //{
      //result<<" - Best!";
    //}
  }
  result<<"\n****************************************************";

  return result.str();
} // End fitnessFeasibleToString





/** toString */
const std::string Population::toString() const 
{
  std::ostringstream result;

//  result<<"Best ID: "<<calcBestIndex();

  // If sub-populations exist, print those
  if(subPopulations_.size() > 0) 
  {
    for(unsigned int i=0;i<subPopulations_.size();i++) 
    {
      result<<"\n\nSub-Population "<<i<<":";

      for(unsigned int j=0;j<subPopulations_.at(i).trajectories_.size();j++) 
      {
        result<<"\n\nTrajectory "<<subPopulations_.at(i).trajectories_.at(j).msg_.id<<": "<<subPopulations_.at(i).trajectories_.at(j).toString();
      }
    }
  }

  // Otherwise, print population as a whole
  else 
  {
    for(unsigned int i=0;i<trajectories_.size();i++) 
    {
      ////ROS_INFO("Population::toString i: %i", (int)i);
      result<<"\nTrajectory "<<i<<": "<<trajectories_.at(i).toString();
    }
  }

  return result.str();
} //End toString




//Return a message of type ramp_msgs::Population to be sent to the trajectory viewer 
ramp_msgs::Population Population::populationMsg() const
{
  ramp_msgs::Population msg;
  
  for(int i=0; i<trajectories_.size(); i++) {
    msg.population.push_back(trajectories_.at(i).msg_);
  }
  
  msg.best_id = calcBestIndex();
  return msg;
}
