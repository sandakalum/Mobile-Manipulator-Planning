#include <moveit_tmp/task_planner.h>

namespace moveit_tmp
{
TaskPlanner::TaskPlanner() {}

std::string TaskPlanner::runCommand(std::string cmd)
{
  std::string data;
  FILE* stream;
  char buffer[1000];
  stream = popen(cmd.c_str(), "r");
  while (fgets(buffer, 1000, stream) != nullptr)
    data.append(buffer);
  pclose(stream);
  return data;
}

bool TaskPlanner::plan(std::vector<Action>& result)
{
  result.clear();

  // prepare the planner command line
  std::string str = planner_command_;
  std::size_t dit = str.find("DOMAIN");
  if (dit != std::string::npos)
    str.replace(dit, 6, domain_path_);
  std::size_t pit = str.find("PROBLEM");
  if (pit != std::string::npos)
    str.replace(pit, 7, problem_path_);
  std::string commandString = str + " > " + data_path_ + "plan.pddl";

  // call the planer
  ROS_INFO("(%s) Running: %s", ros::this_node::getName().c_str(),
           commandString.c_str());
  std::string plan = runCommand(commandString.c_str());
  ROS_INFO("(%s) Planning complete", ros::this_node::getName().c_str());

  // check the planner solved the problem
  std::ifstream planfile;
  planfile.open((data_path_ + "plan.pddl").c_str());
  std::string line;

  bool solved = false;
  while (not solved and std::getline(planfile, line))
  {
    if (line.compare("ff: found legal plan as follows") == 0)
    {
      solved = true;
    }
  }
  // Parse the solved plan
  if (solved)
  {
    // actions look like this:
    // step    0: got_place C1
    //         1: find_object V1 C1
    // plan cost: XX
    while (std::getline(planfile, line))
    { // Move to the beginning of the plan
      if (line.substr(0, 4) == "step")
      {
        line = line.substr(4); // Remove the step
        break;
      }
    }
    // First iteration line will be like   0: got_place C1
    while (line.find("plan cost") == line.npos and
           line.find("time spend") == line.npos and line.size() > 0)
    {
      std::stringstream ss(line); // To trim whitespaces
      std::string aux;
      Action action;

      // Read the action number X:
      ss >> aux;
      if (aux != "")
      {
        // Read the action name:
        ss >> aux;
        std::transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
        action.name = aux;

        // Read parameters
        while (ss >> aux)
        {
          std::transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
          action.parameters.push_back(aux);
        }

        result.push_back(action);
      }

      std::getline(planfile, line);
    }
  }
  planfile.close();

  if (!solved)
    ROS_INFO("(%s) Plan was unsolvable.", ros::this_node::getName().c_str());
  else
    ROS_INFO("(%s) Plan was solved.", ros::this_node::getName().c_str());

  return solved;
}
} // namespace moveit_tmp
