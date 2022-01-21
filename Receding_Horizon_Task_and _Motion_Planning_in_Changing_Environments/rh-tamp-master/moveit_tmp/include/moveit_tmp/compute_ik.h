#ifndef COMPUTE_IK_H
#define COMPUTE_IK_H

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit_tmp
{

class ComputeIK
{
public:
  ComputeIK();

  bool compute(const planning_scene::PlanningSceneConstPtr& from,
               planning_scene::PlanningScenePtr& to,
               std::vector<std::string>& collision_objects);

  void setIKFrame(const geometry_msgs::PoseStamped& pose) { ik_frame_ = pose; }
  void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);

  void setTargetPose(const geometry_msgs::PoseStamped& pose)
  {
    target_pose_ = pose;
  }
  void setTargetPose(const Eigen::Isometry3d& pose,
                     const std::string& frame = "");

  void setGroup(const std::string& group) { group_ = group; }
  void setEndEffector(const std::string& eef) { eef_ = eef; }

  void setMaxIKSolutions(unsigned int n) { max_solutions_ = n; }
  void setTimeout(double timeout = 0.1) { timeout_ = timeout; }

private:
  bool convertDoublesToEigen(std::vector<double> values,
                             Eigen::Isometry3d& transform);

  bool isTargetPoseColliding(
      const planning_scene::PlanningScenePtr& scene, Eigen::Isometry3d pose,
      const robot_model::LinkModel* link,
      collision_detection::CollisionResult* collision_result = nullptr);

  // TODO: aggiunto
  bool isTargetPoseColliding(
      const planning_scene::PlanningSceneConstPtr& scene,
      collision_detection::CollisionResult* collision_result = nullptr);

  std::string listCollisionPairs(
      const collision_detection::CollisionResult::ContactMap& contacts,
      const std::string& separator);

  void listCollisionObjects(
      const collision_detection::CollisionResult::ContactMap& contacts,
      std::vector<std::string>& objects);

  std::string group_;
  std::string eef_;

  geometry_msgs::PoseStamped ik_frame_;
  geometry_msgs::PoseStamped target_pose_;

  unsigned int max_solutions_;
  double timeout_;
};
} // namespace moveit_tmp

#endif // COMPUTE_IK_H
