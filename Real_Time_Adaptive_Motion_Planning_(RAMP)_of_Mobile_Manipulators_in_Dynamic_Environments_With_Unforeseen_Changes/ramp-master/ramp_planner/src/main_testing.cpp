#include "ros/ros.h"
#include "planner.h"
 

Utility utility;


Planner             my_planner; 
int                 id;
MotionState         start, goal;
std::vector<Range>  ranges;
int                 population_size;
int                 gensBeforeCC;
bool                sub_populations;
bool                modifications;
bool                evaluations;
bool                seedPopulation;
bool                errorReduction;
double              t_cc_rate;
double              t_pc_rate;
int                 num_obs;
std::vector<std::string> ob_topics;
std::vector<tf::Transform> ob_tfs;


// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  
  for(unsigned int i=0;i<dof_min.size();i++) {
    Range temp(dof_min.at(i), dof_max.at(i));
    ranges.push_back(temp); 
  }

} // End initDOF



// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) 
{
  for(unsigned int i=0;i<s.size();i++) {
    start.msg_.positions.push_back(s.at(i));
    goal.msg_.positions.push_back(g.at(i));

    start.msg_.velocities.push_back(0);
    goal.msg_.velocities.push_back(0);

    start.msg_.accelerations.push_back(0);
    goal.msg_.accelerations.push_back(0);

    start.msg_.jerks.push_back(0);
    goal.msg_.jerks.push_back(0);
  }
} // End initStartGoal



void loadObstacleTF()
{
  std::ifstream ifile("/home/mahmoud/NetBeansProjects/ROSProject/ros_ws/src/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}


/** Loads all of the ros parameters from .yaml 
 *  Calls initDOF, initStartGoal */
void loadParameters(const ros::NodeHandle handle) 
{
  std::cout<<"\nLoading parameters\n";
  std::cout<<"\nHandle NS: "<<handle.getNamespace();

  std::string key;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) 
  {
    handle.getParam("robot_info/id", id);
  }
  else 
  {
    ROS_ERROR("Did not find parameter robot_info/id");
  }


  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    initDOF(dof_min, dof_max);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }


  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> p_start;
    std::vector<float> p_goal;
    handle.getParam("robot_info/start", p_start);
    handle.getParam("robot_info/goal",  p_goal );
    initStartGoal(p_start, p_goal);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
  }



  if(handle.hasParam("ramp/population_size")) 
  {
    handle.getParam("ramp/population_size", population_size);
    std::cout<<"\npopulation_size: "<<population_size;
  }

  
  if(handle.hasParam("ramp/sub_populations")) 
  {
    handle.getParam("ramp/sub_populations", sub_populations);
    std::cout<<"\nsub_populations: "<<sub_populations;
  }
  
  if(handle.hasParam("ramp/modifications")) 
  {
    handle.getParam("ramp/modifications", modifications);
    std::cout<<"\nmodifications: "<<modifications;
  }
  
  if(handle.hasParam("ramp/evaluations")) 
  {
    handle.getParam("ramp/evaluations", evaluations);
    std::cout<<"\nevaluations: "<<evaluations;
  }
  
  if(handle.hasParam("ramp/seed_population")) 
  {
    handle.getParam("ramp/seed_population", seedPopulation);
    std::cout<<"\nseed_population: "<<seedPopulation;
  }
  
  if(handle.hasParam("ramp/gens_before_control_cycle")) 
  {
    handle.getParam("ramp/gens_before_control_cycle", gensBeforeCC);
    std::cout<<"\ngens_before_control_cycle: "<<gensBeforeCC;
  }
  
  if(handle.hasParam("ramp/fixed_control_cycle_rate")) 
  {
    handle.getParam("ramp/fixed_control_cycle_rate", t_cc_rate);
    ROS_INFO("t_cc_rate: %f", t_cc_rate);
  }
  
  if(handle.hasParam("ramp/planning_cycle_rate")) {
    handle.getParam("ramp/planning_cycle_rate", t_pc_rate);
    ROS_INFO("t_pc_rate: %f", t_pc_rate);
  }
  
  if(handle.hasParam("ramp/error_reduction")) 
  {
    handle.getParam("ramp/error_reduction", errorReduction);
    ROS_INFO("errorReduction: %s", errorReduction ? "True" : "False");
  }

  if(handle.hasParam("ramp/num_of_obstacles"))
  {
    handle.getParam("ramp/num_of_obstacles", num_obs);
    ROS_INFO("num_of_obstacles: %i", num_obs);
  }


  if(handle.hasParam("ramp/obstacle_topics"))
  {
    handle.getParam("ramp/obstacle_topics", ob_topics);
    ROS_INFO("ob_topics.size(): %i", (int)ob_topics.size());
    for(int i=0;i<ob_topics.size();i++)
    {
      ROS_INFO("ob_topics[%i]: %s", i, ob_topics.at(i).c_str());
    }
  }



  std::cout<<"\n------- Done loading parameters -------\n";
    std::cout<<"\n  ID: "<<id;
    std::cout<<"\n  Start: "<<start.toString();
    std::cout<<"\n  Goal: "<<goal.toString();
    std::cout<<"\n  Ranges: ";
    for(uint8_t i=0;i<ranges.size();i++) 
    {
      std::cout<<"\n  "<<i<<": "<<ranges.at(i).toString();
    }
  std::cout<<"\n---------------------------------------";
}

int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  std::cout<<"\nHandle namespace: "<<handle.getNamespace();
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1, &Planner::updateCallback, &my_planner);
  ros::Subscriber sub_sc_ = handle.subscribe("obstacles", 1, &Planner::sensingCycleCallback, &my_planner);


  // Load ros parameters
  loadParameters(handle);

  loadObstacleTF();

  ROS_INFO("Parameters loaded. Please review them and press Enter to continue");
  std::cin.get();

 
  /** Initialize the Planner's handlers */ 
  my_planner.init(id, handle, start, goal, ranges, population_size, sub_populations, ob_tfs, gensBeforeCC, t_pc_rate, t_cc_rate, errorReduction); 
  my_planner.modifications_   = modifications;
  my_planner.evaluations_     = evaluations;
  my_planner.seedPopulation_  = seedPopulation;

  std::cout<<"\nStart: "<<my_planner.start_.toString();
  std::cout<<"\nGoal: "<<my_planner.goal_.toString();

  
 
 
  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();


  
  //****MotionState exp_results = my_planner.findAverageDiff();
  //****std::cout<<"\n\nAverage Difference: "<<exp_results.toString();
  
  
  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}

