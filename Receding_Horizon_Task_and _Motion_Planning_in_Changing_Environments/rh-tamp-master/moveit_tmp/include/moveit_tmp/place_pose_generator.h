#ifndef PLACE_POSE_GENERATOR_H
#define PLACE_POSE_GENERATOR_H

#include <cmath>
#include <ctime>
#include <memory>
#include <random>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit_tmp
{

class PlacePoseGenerator
{
public:
  PlacePoseGenerator();

  bool compute(const planning_scene::PlanningSceneConstPtr& scene,
               const std::string object_id, const std::string surface_id,
               const Eigen::Isometry3d& p_min, const Eigen::Isometry3d& p_max,
               geometry_msgs::PoseStamped& target_pose);

  void setFixedLink(const std::string fixed_link) { fixed_link_ = fixed_link; }

private:
  std::default_random_engine rng_;

  int max_attempts_;

  std::string fixed_link_;

  const std::string PLACEHOLDER_ = "placeholder";
};

} // namespace moveit_tmp

#endif // PLACE_POSE_GENERATOR_H
