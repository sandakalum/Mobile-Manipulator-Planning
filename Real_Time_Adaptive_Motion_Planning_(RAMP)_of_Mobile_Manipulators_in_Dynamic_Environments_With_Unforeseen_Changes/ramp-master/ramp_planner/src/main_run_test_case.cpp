#include "planner.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
 

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
int                 pop_type;
TrajectoryType      pt;
std::vector<std::string> ob_topics;


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
  
  if(handle.hasParam("ramp/pop_traj_type")) 
  {
    handle.getParam("ramp/pop_traj_type", pop_type);
    ROS_INFO("pop_type: %s", pop_type ? "Partial Bezier" : "All Straight");
    switch (pop_type) 
    {
      case 0:
        pt = HOLONOMIC;
        break;
      case 1:
        pt = HYBRID;
        break;
    }
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





enum Group 
{
  EXTERIOR = 0,
  INTERIOR = 1,
  CRITICAL = 2,
  MIX      = 3
};


struct ObInfo
{
  double x;
  double y;
  double v;
  double w;
  double relative_direction;
  double d;
  ramp_msgs::Obstacle msg;
  
  bool faster;
};

struct TestCase 
{
  ros::Time t_begin;
  std::vector<ObInfo> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<RampTrajectory> ob_trjs;
  Group group;
  int history;

  bool success;
};

ramp_msgs::Obstacle getStaticOb(ramp_msgs::Obstacle ob)
{
  ramp_msgs::Obstacle result = ob; 

  result.ob_ms.velocities[0] = 0;
  result.ob_ms.velocities[1] = 0;
  result.ob_ms.velocities[2] = 0;

  return result;
}

ObInfo generateObInfoSimple(const MotionState robot_state)
{
  ObInfo result;

  Range v(0,  0.33);
  Range w(0,  PI/2.f);

  result.v = v.random();
  result.w = w.random();
  result.relative_direction = 0;

  result.x = robot_state.msg_.positions[0];
  result.y = robot_state.msg_.positions[1] + 1;

  result.d = 1;
  
  return result;
}

ObInfo generateObInfo(const MotionState robot_state)
{
  ObInfo result;

  Range dist(0, 3.5);

  Range v(0, 0.5);
  Range w       (0      ,  PI/2.f);
  Range rela_dir(PI/6.f ,  5.f*PI/6.f);
  
  
  result.v                  = v.random();
  result.w                  = w.random();
  result.relative_direction = rela_dir.random();
  
  
  result.d    = dist.random();
  double ob_x = robot_state.msg_.positions[0] + result.d*cos(result.relative_direction);
  double ob_y = robot_state.msg_.positions[1] + result.d*sin(result.relative_direction);

  if(ob_x > 3.5)
    ob_x = 3.5;
  else if(ob_x < 0)
    ob_x = 0.;
  if(ob_y > 3.5)
    ob_y = 3.5;
  else if(ob_y < 0)
    ob_y = 0.;

  result.x = ob_x;
  result.y = ob_y;

  result.faster = result.v > sqrt(pow(robot_state.msg_.velocities[0],2) + pow(robot_state.msg_.velocities[1],2));

  return result;
}



/*
 * p_x and p_y are the position values,                 range: [0, 3.5]
 * v_mag is the magnitude of the linear velocity,       range: [0, 0.25]
 * v_direction is the direction of the linear velocity, range: [0, pi]
 * w is the angular velocity (not a vector),            range: [-pi/4, pi/4]
 */
const ramp_msgs::Obstacle buildObstacleMsg(const double& p_x, const double& p_y, const double& v_mag, const double& v_direction, const double& w)
{
  ROS_INFO("p_x: %f p_y: %f, v_mag: %f v_direction: %f w: %f", p_x, p_y, v_mag, v_direction, w);

  ramp_msgs::Obstacle result;

  // odom_msg describes the obstacle's position and velocity
  nav_msgs::Odometry odom_msg;

  // Set the positions
  result.ob_ms.positions.push_back(p_x);
  result.ob_ms.positions.push_back(p_y);
  result.ob_ms.positions.push_back(v_direction);
  
  // For linear velocity, calculate x and y components
  double v_x = v_mag*cos(v_direction);
  double v_y = v_mag*sin(v_direction);

  // Set velocities
  result.ob_ms.velocities.push_back(v_mag*cos(v_direction));
  result.ob_ms.velocities.push_back(v_mag*sin(v_direction));
  result.ob_ms.velocities.push_back(w);

  return result;
}


TestCase generateTestCase(const MotionState robot_state, double inner_r, double outter_r, int num_obs)
{
  TestCase result;

  Range history(0,20);

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    //ObInfo temp = generateObInfo(robot_state);
    ObInfo temp = generateObInfoSimple(robot_state);

    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v, temp.relative_direction, temp.w);
    
    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);

    result.history = history.random();
  }

  return result;
}



MotionState getGoal(const MotionState init, const double dim)
{
  ROS_INFO("getGoal init: %s", init.toString().c_str());

  double r = sqrt( pow(dim,2) * 2 );
  ROS_INFO("r: %f", r);
  double x = init.msg_.positions[0] + r*cos(PI/4.f);
  double y = init.msg_.positions[1] + r*sin(PI/4.f);

  MotionState result;
  result.msg_.positions.push_back(x);
  result.msg_.positions.push_back(y);
  result.msg_.positions.push_back(init.msg_.positions[2]);

  ROS_INFO("getGoal result: %s", result.toString().c_str());
  return result;
}



void pubObTrj(const ros::TimerEvent e, const TestCase tc)
{
  ros::Duration d_elapsed = ros::Time::now() - tc.t_begin;

  int index = d_elapsed.toSec() * 10;

  trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[0].msg_.trajectory.points[index];

  // Build new obstacle msg
  ramp_msgs::Obstacle ob = buildObstacleMsg(p.positions[0], p.positions[1], tc.obs[0].v, p.positions[2], tc.obs[0].w);
}


int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  // Load ros parameters and obstacle transforms
  loadParameters(handle);

  my_planner.ranges_ = ranges;
  
  ROS_INFO("Parameters loaded. Please review them and press Enter to continue");
  //std::cin.get();

  ros::Rate r(100);
  
  ros::Subscriber sub_sc_     = handle.subscribe("obstacles", 1, &Planner::sensingCycleCallback,  &my_planner);
  ros::Subscriber sub_update_ = handle.subscribe("update",    1, &Planner::updateCallback,        &my_planner);

  ros::ServiceClient client_reset = handle.serviceClient<std_srvs::Empty>("reset_positions");
  std_srvs::Empty reset_srv;


  ros::Timer ob_trj_timer;
  
  int num_tests = 42;
  int num_successful_tests = 0;
  std::vector<int> num_generations;
  std::vector<TestCase> test_cases;

  ros::Duration d_test_case_thresh(20);
  
  ROS_INFO("Before for loop");
  ROS_INFO("Press Enter to start");
  std::cin.get();

  MotionState start;
  start.msg_.positions.push_back(0);
  start.msg_.positions.push_back(0);
  start.msg_.positions.push_back(PI/4.f);
  start.msg_.velocities.push_back( 0.176 );
  start.msg_.velocities.push_back( 0.176 );
  start.msg_.velocities.push_back( 0 );


  MotionState goal;
  goal.msg_.positions.push_back(2);
  goal.msg_.positions.push_back(2);
  goal.msg_.positions.push_back(PI/4.f);



  ros::Duration d(1);
  for(int i=0;i<num_tests;i++)
  {
    ROS_INFO("i: %i", i);
    
    // Test case done, stop publishing obs
    my_planner.h_parameters_.setTestCase(false);

    // Wait for test case to be generated
    bool tc_generated = false;
    while(!tc_generated)
    {
      handle.getParam("/ramp/tc_generated", tc_generated);
      r.sleep();
      ros::spinOnce();
    }

    ROS_INFO("Run: TC is generated, Preparing RAMP for test case");

    MotionState initial_state;
    my_planner.randomMS(initial_state);

    /*
     * Set the obstacle transformations to be the initial position
     */

    /** Initialize the Planner */ 
    my_planner.init(id, handle, start, goal, ranges, population_size, sub_populations, pt, gensBeforeCC, 
        t_pc_rate, t_cc_rate, errorReduction);
    my_planner.modifications_   = modifications;
    my_planner.evaluations_     = evaluations;
    my_planner.seedPopulation_  = seedPopulation;

    ROS_INFO("Planner initialized");
    ROS_INFO("Start: %s", my_planner.start_.toString().c_str());
    ROS_INFO("Goal: %s", my_planner.goal_.toString().c_str());

    /*
     * Prep test
     */
    // Initialize a population, perform a control cycle, and get the point at the end of current trajectory
    auto p_next_cc = my_planner.prepareForTestCase();

    ROS_INFO("Done with prepareForTestCase");


    // Start publishing obstacle info
    my_planner.h_parameters_.setTestCase(true); 

    /*
     * Run planner
     */
    my_planner.goTest(d_test_case_thresh.toSec());
    
    
    // Test case done, stop publishing obs
    my_planner.h_parameters_.setTestCase(false);

    // Reset Stage positions
    client_reset.call(reset_srv);
  }


  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}
