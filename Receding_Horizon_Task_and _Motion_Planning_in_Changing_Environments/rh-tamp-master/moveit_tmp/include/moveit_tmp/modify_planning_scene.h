#ifndef MODIFYPLANNINGSCENE_H
#define MODIFYPLANNINGSCENE_H

#include <map>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit_tmp
{
class ModifyPlanningScene
{
public:
  typedef std::vector<std::string> Names;

  ModifyPlanningScene();

  /// attach or detach a list of objects to the given link
  void attachObjects(const Names& objects, const std::string& attach_link,
                     bool attach);

  /// conviency methods accepting a single object name
  inline void attachObject(const std::string& object, const std::string& link)
  {
    attachObjects(Names({object}), link, true);
  }

  inline void detachObject(const std::string& object, const std::string& link)
  {
    attachObjects(Names({object}), link, false);
  }

  inline void attachObjects(const Names& objects, const std::string& link)
  {
    attachObjects(objects, link, true);
  }

  inline void detachObjects(const Names& objects, const std::string& link)
  {
    attachObjects(objects, link, false);
  }

  /// allow / forbid collisions for each combination of pairs in first and
  /// second lists
  void allowCollisions(const Names& first, const Names& second, bool allow);

  /// allow / forbid collisions for pair (first, second)
  void allowCollisions(const std::string& first, const std::string& second,
                       bool allow)
  {
    allowCollisions(Names{first}, Names{second}, allow);
  }

  /// allow / forbid all collisions for given object
  void allowCollisions(const std::string& object, bool allow)
  {
    allowCollisions(Names({object}), Names(), allow);
  }

  /// conveniency method accepting std::string and an arbitrary container of
  /// names
  inline void allowCollisions(const std::string& first, const Names& second,
                              bool enable_collision)
  {
    allowCollisions(Names({first}), second, enable_collision);
  }

  /// conveniency method accepting std::string and JointModelGroup
  void allowCollisions(const std::string& first,
                       const moveit::core::JointModelGroup& jmg, bool allow);

  void moveObject(const std::string& object, const geometry_msgs::PoseStamped& pose);

  void moveObject(const std::string& object, const geometry_msgs::Pose& pose);

  void apply(const planning_scene::PlanningSceneConstPtr& from,
             planning_scene::PlanningScenePtr& to);

protected:
  // list of objects to attach (true) / detach (false) to a given link
  std::map<std::string, std::pair<Names, bool>> attach_objects_;

  // list of objects to mutually
  struct CollisionMatrixPairs
  {
    Names first;
    Names second;
    bool allow;
  };

  std::list<CollisionMatrixPairs> collision_matrix_edits_;

  std::vector<std::pair<std::string, geometry_msgs::PoseStamped>> move_objects_;

  void
  attachObjects(planning_scene::PlanningScene& scene,
                const std::pair<std::string, std::pair<Names, bool>>& pair);

  void allowCollisions(planning_scene::PlanningScene& scene,
                       const CollisionMatrixPairs& pairs);

  void moveObjects(planning_scene::PlanningScene& scene,
                   const std::pair<std::string, geometry_msgs::PoseStamped>& pair);
};
} // namespace moveit_tmp

#endif // MODIFYPLANNINGSCENE_H
