#ifndef PLACE_H
#define PLACE_H

#include <moveit_tmp/action_planner.h>
#include <moveit_tmp/compute_ik.h>
#include <moveit_tmp/modify_planning_scene.h>
#include <moveit_tmp/move.h>
#include <moveit_tmp/move_cartesian.h>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit_tmp
{
class Place : public ActionPlanner
{
public:
  Place(const std::string& name, const PlannerPtr& planner);

  bool reason(const planning_scene::PlanningSceneConstPtr& planning_scene,
              const std::string& object_name,
              const std::vector<Eigen::Isometry3d>& target_poses,
              planning_scene::PlanningScenePtr& goal,
              std::vector<std::string>& collision_objects) override;

  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
            const std::string object_name,
            const std::vector<double>& joint_values,
            ActionPipeline& pipeline) override;

private:
  ComputeIK ik_;
  ModifyPlanningScene mps_;
  PlannerPtr planner_;
  Move move_;
};
} // namespace moveit_tmp

#endif // PLACE_H
