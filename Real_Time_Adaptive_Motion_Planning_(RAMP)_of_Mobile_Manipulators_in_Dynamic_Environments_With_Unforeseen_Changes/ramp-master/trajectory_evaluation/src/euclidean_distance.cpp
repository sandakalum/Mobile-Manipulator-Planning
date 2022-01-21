#include "../include/euclidean_distance.h"


/** This method finds the euclidean distance from the last trajectory point to the goal */
const double EuclideanDistance::perform() {
  double result=0;

  trajectory_msgs::JointTrajectoryPoint end_point = trajectory_.trajectory.points.at(trajectory_.trajectory.points.size() - 1);


  std::vector<double> v_end_point;
  v_end_point.push_back(end_point.positions.at(0));
  v_end_point.push_back(end_point.positions.at(1));

  std::vector<double> v_goal;
  v_goal.push_back(goal_.positions.at(0));
  v_goal.push_back(goal_.positions.at(1));

  result = utility_.positionDistance(v_end_point, v_goal);
  
  return result;
}
