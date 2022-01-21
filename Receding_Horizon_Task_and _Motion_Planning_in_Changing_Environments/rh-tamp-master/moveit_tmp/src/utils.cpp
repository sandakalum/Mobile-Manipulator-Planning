#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

bool convertDoublesToEigen(std::vector<double> values,
                           Eigen::Isometry3d& transform)
{
  if (values.size() == 6)
  {
    // This version is correct RPY
    Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

    transform =
        Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;

    return true;
  }
  else if (values.size() == 7)
  {
    // Quaternion
    transform = Eigen::Translation3d(values[0], values[1], values[2]) *
                Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid number of doubles provided for transform, size="
                     << values.size());
    return false;
  }
}
