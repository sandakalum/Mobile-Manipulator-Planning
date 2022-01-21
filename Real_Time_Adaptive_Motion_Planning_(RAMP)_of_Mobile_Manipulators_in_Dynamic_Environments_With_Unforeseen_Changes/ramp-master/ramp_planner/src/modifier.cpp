#include "modifier.h"


Modifier::Modifier(const ros::NodeHandle& h, const unsigned int n) : num_ops(n), move_dir_(101), move_dist_(101)
{
  h_mod_req_ = new ModificationRequestHandler(h);
}

Modifier::~Modifier() 
{
  if(h_mod_req_ != 0) 
  {
    delete h_mod_req_;
    h_mod_req_ = 0;
  }
}




/** This method returns a random operator */
const std::string Modifier::getOperator() const 
{
  std::string result;  
  
  // First, randomly select an operator
  unsigned int op = rand() % num_ops;

  // Assign the correct name for the operator
  switch(op) 
  {

    // Insert
    case 0:
      result = "insert";
      break;

    // Delete
    case 1:
      result = "delete";
      break;

    // Change
    case 2:
      result = "change";
      break;

    // Swap
    case 3:
      result = "swap";
      break;

    // Crossover
    case 4:
      result = "crossover";
      break;

    // Move/repair
    case 5:
      result = "move";
      break;
  }
 
  return result;
} // End getOperator


/** This method generates the random paths to use for the modification operator passed in as op argument */
const std::vector<int> Modifier::getTargets(const std::string& op, const Population& pop) 
{
  //ROS_INFO("In Modifier::getTargets");
  //////ROS_INFO("pop.paths_.size(): %i", (int)pop.paths_.size());
  //////ROS_INFO("pop.trajec.size(): %i", (int)pop.trajectories_.size());
  //////ROS_INFO("Pop: %s", pop.toString().c_str());
  //////ROS_INFO("op: %s", op.c_str());
  std::vector<int> result;

  // Get random path(s) to modify
  unsigned int i_p1 = rand() % pop.paths_.size();
  
  // Push on i_p1
  result.push_back(i_p1);


  // If crossover, get a second path
  if(op == "crossover") 
  {
    unsigned int i_p2;
    do { i_p2 = rand() % pop.paths_.size(); } 
    while (i_p1 == i_p2);
  
    // Push on i_p1
    result.push_back(i_p2);
  } // end if crossover 


  //ROS_INFO("Exiting Modifier::getTargets");
  return result;
} // End getTargets



/** 
 * This method builds a ModificationRequest srv 
 * For stop operator, the path can be retreived from srv
 * */
void Modifier::buildModificationRequest(const Population& pop, bool imminent_collision, ramp_msgs::ModificationRequest& result)
{

  // Push the target paths onto the modification request
  std::vector<int> targets;

  if(!imminent_collision || (move_dist_ > 100 && move_dir_ > 100))
  {
    ////ROS_INFO("Modifier: No IC");
    result.request.op = getOperator();
    targets           = getTargets(result.request.op, pop);
  }
  else
  {
    ////ROS_INFO("Modifier: Calling Move because IC exists");
    result.request.op = "move";
    targets.push_back(pop.calcBestIndex());  
  }


  //////ROS_INFO("targets.size(): %i", (int)targets.size());
  for(unsigned int i=0;i<targets.size();i++) 
  {
    //std::cout<<"\nTarget index: "<<targets.at(i);
    //std::cout<<"\nPath "<<targets.at(i)<<" size: "<<pop.paths_.at(targets.at(i)).size()<<"\n";
    result.request.paths.push_back(
        pop.paths_.at(targets.at(i)).buildPathMsg());
  }

  result.request.move_dir = move_dir_;
  result.request.move_dist = move_dist_;

} // End buildModificationRequest




/** This method performs all the tasks for path modification */
const std::vector<Path> Modifier::perform(const Population& pop, bool imminent_collision) 
{
  //////ROS_INFO("In Modifier::perform");
  std::vector<Path> result;
 
  // Build a modification request srv 
  ros::Time t_b = ros::Time::now();
  ramp_msgs::ModificationRequest mr;
  buildModificationRequest(pop, imminent_collision, mr); 
  //////ROS_INFO("ModificationResult built, pop size: %i # of paths: %i", (int)pop.size(), (int)mr.response.mod_paths.size()); 

  //////ROS_INFO("Requesting modification");
  // If the request was successful
  if(h_mod_req_->request(mr)) 
  {
    //////ROS_INFO("Got modification");
    ros::Time t_m = ros::Time::now();
    
    // Push on the modified paths
    for(unsigned int i=0;i<mr.response.mod_paths.size();i++) 
    {
      Path temp(mr.response.mod_paths.at(i));
      result.push_back(temp);
    }
  } // end inner if 

  //////ROS_INFO("Exiting Modifier::perform");
  return result;
}
