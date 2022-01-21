#ifndef ACTION_CACHE_H
#define ACTION_CACHE_H

#include <map>
#include <string>
#include <vector>

namespace moveit_tmp
{
class ActionCache
{
public:
  using ActionID = std::tuple<std::string, std::string, std::string>;
  using PlannerMap = std::map<ActionID, std::vector<double>>;

  ActionCache();

  bool retrieve(const ActionID& id, std::vector<double>& joints);

  void insert(const ActionID& id, const std::vector<double>& joints);

  void remove(const ActionID& id);

  void reset();

private:
  PlannerMap cache_;
};
} // namespace moveit_tmp

#endif // ACTION_CACHE_H
