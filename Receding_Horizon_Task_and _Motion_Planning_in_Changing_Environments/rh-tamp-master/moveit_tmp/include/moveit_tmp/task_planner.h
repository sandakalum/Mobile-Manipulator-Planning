#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

//#include <streambuf>
//#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace moveit_tmp
{
struct Action
{
  std::string name;
  std::vector<std::string> parameters;
};

std::ostream& operator<<(std::ostream& stream, Action const& action)
{
  stream << action.name;

  for (int i = 0; i < action.parameters.size(); i++)
  {
    stream << " " << action.parameters[i];
  }
  return stream;
}

class TaskPlanner
{
public:
  TaskPlanner();

  void setDomainPath(const std::string& path) { domain_path_ = path; }

  void setProblemPath(const std::string& path) { problem_path_ = path; }

  void setPath(const std::string& path) { data_path_ = path; }

  void setPlannerCommand(const std::string& command)
  {
    planner_command_ = command;
  }

  bool plan(std::vector<Action>& result);

private:
  std::string runCommand(std::string cmd);

  std::string planner_command_;
  std::string domain_path_;
  std::string problem_path_;
  std::string data_path_;
};
} // namespace moveit_tmp

#endif // TASK_PLANNER_H
