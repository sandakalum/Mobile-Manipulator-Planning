#include <moveit_tmp/action_cache.h>

#include <iostream>

namespace moveit_tmp
{
ActionCache::ActionCache() {}

bool ActionCache::retrieve(const ActionID& id, std::vector<double>& joints)
{
  PlannerMap::iterator it = cache_.find(id);
  if (it == cache_.end())
    return false;

  joints = it->second;
  return true;
}

void ActionCache::insert(const ActionID& id, const std::vector<double>& joints)
{
  cache_[id] = joints;

  std::cout << "CACHE ADD: " << std::get<0>(id) << " " << std::get<1>(id) << " "
            << std::get<2>(id) << std::endl;
}

void ActionCache::remove(const ActionID& id)
{
  cache_.erase(id);
  std::cout << "CACHE REMOVE: " << std::get<0>(id) << " " << std::get<1>(id)
            << " " << std::get<2>(id) << std::endl;
}

void ActionCache::reset() { cache_.clear(); }
} // namespace moveit_tmp
