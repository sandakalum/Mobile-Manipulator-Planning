#ifndef ACTION_PLANNER_H
#define ACTION_PLANNER_H

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_tmp/planner.h>
#include <string>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit_tmp
{

typedef std::pair<planning_scene::PlanningScenePtr,
                  robot_trajectory::RobotTrajectoryPtr>
    Step;
typedef std::vector<Step> ActionPipeline;

class ActionPlanner
{
public:
  ActionPlanner(const std::string& name);

  virtual ~ActionPlanner() {}

  void setIKFrame(const geometry_msgs::PoseStamped& pose) { ik_frame_ = pose; }
  void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);

  void setGroup(const std::string& group) { group_ = group; }

  void setEndEffector(const std::string& eef) { eef_ = eef; }

  virtual bool
  reason(const planning_scene::PlanningSceneConstPtr& planning_scene,
         const std::string& object_name,
         const std::vector<Eigen::Isometry3d>& target_poses,
         planning_scene::PlanningScenePtr& goal,
         std::vector<std::string>& collision_objects) = 0;

  virtual bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const std::string object_name,
                    const std::vector<double>& joint_values,
                    ActionPipeline& pipeline) = 0;

protected:
  std::string name_;
  std::string group_;
  std::string eef_;
  geometry_msgs::PoseStamped ik_frame_;
};
} // namespace moveit_tmp
#endif // ACTION_PLANNER_H
