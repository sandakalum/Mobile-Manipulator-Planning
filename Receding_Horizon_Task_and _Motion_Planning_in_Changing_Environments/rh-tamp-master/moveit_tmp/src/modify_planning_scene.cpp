#include <moveit_tmp/modify_planning_scene.h>

namespace moveit_tmp
{
ModifyPlanningScene::ModifyPlanningScene() {}

void ModifyPlanningScene::attachObjects(const Names& objects,
                                        const std::string& attach_link,
                                        bool attach)
{
  auto it_inserted = attach_objects_.insert(
      std::make_pair(attach_link, std::make_pair(Names(), attach)));
  Names& o = it_inserted.first->second.first;
  o.insert(o.end(), objects.begin(), objects.end());
}

void ModifyPlanningScene::allowCollisions(const Names& first,
                                          const Names& second, bool allow)
{
  collision_matrix_edits_.push_back(
      CollisionMatrixPairs({first, second, allow}));
}

void ModifyPlanningScene::allowCollisions(
    const std::string& first, const moveit::core::JointModelGroup& jmg,
    bool allow)
{
  const auto& links = jmg.getLinkModelNamesWithCollisionGeometry();
  if (!links.empty())
    allowCollisions(Names({first}), links, allow);
}

void ModifyPlanningScene::attachObjects(
    planning_scene::PlanningScene& scene,
    const std::pair<std::string, std::pair<Names, bool>>& pair)
{
  moveit_msgs::AttachedCollisionObject obj;
  obj.link_name = pair.first;
  bool attach = pair.second.second;

  obj.object.operation = attach ? (int8_t)moveit_msgs::CollisionObject::ADD
                                : (int8_t)moveit_msgs::CollisionObject::REMOVE;
  for (const std::string& name : pair.second.first)
  {
    obj.object.id = name;

    scene.processAttachedCollisionObjectMsg(obj);
  }
}

void ModifyPlanningScene::moveObject(const std::string& object,
                                     const geometry_msgs::PoseStamped& pose)
{
  move_objects_.push_back(std::make_pair(object, pose));
}

void ModifyPlanningScene::moveObject(const std::string& object,
                                     const geometry_msgs::Pose& pose)
{

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose = pose;

  move_objects_.push_back(std::make_pair(object, pose_stamped));
}

void ModifyPlanningScene::moveObjects(
    planning_scene::PlanningScene& scene,
    const std::pair<std::string, geometry_msgs::PoseStamped>& pair)
{
  moveit_msgs::CollisionObject obj;
  obj.id = pair.first;
  obj.header = pair.second.header;
  obj.primitive_poses.push_back(pair.second.pose);
  obj.operation = moveit_msgs::CollisionObject::MOVE;

  scene.processCollisionObjectMsg(obj);
}

void ModifyPlanningScene::allowCollisions(planning_scene::PlanningScene& scene,
                                          const CollisionMatrixPairs& pairs)
{
  collision_detection::AllowedCollisionMatrix& acm =
      scene.getAllowedCollisionMatrixNonConst();
  bool allow = pairs.allow; // todo semplificare
  if (pairs.second.empty())
  {
    for (const auto& name : pairs.first)
      acm.setEntry(name, allow);
  }
  else
    acm.setEntry(pairs.first, pairs.second, allow);
}

void ModifyPlanningScene::apply(
    const planning_scene::PlanningSceneConstPtr& from,
    planning_scene::PlanningScenePtr& to)
{
  to = from->diff();

  // move objects
  for (const auto& pair : move_objects_)
    moveObjects(*to, pair);

  move_objects_.clear();

  // attach/detach objects
  for (const auto& pair : attach_objects_)
    attachObjects(*to, pair);

  attach_objects_.clear();

  // allow/forbid collisions
  for (const auto& pairs : collision_matrix_edits_)
    allowCollisions(*to, pairs);

  collision_matrix_edits_.clear();
}
} // namespace moveit_tmp
