#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/Population.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Obstacle.h"
#include <chrono>




class CollisionDetection 
{
  public:
  
    /* Struct to hold information about a query */
    struct QueryResult 
    {
      QueryResult() : collision_(false), t_firstCollision_(9999.0f), 
                      i_obstacle_(-1) {}
      bool  collision_;
      float t_firstCollision_;
      int   i_obstacle_;
    };  // End QueryResult


    /***** Constructor and Destructor *****/
    CollisionDetection(); 
    ~CollisionDetection();


    double coll_dist_;

    /***** Methods *****/ 
    void                        init();
    void                        perform(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, QueryResult& result); 
    void                        performNum(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, const double& coll_dist, QueryResult& result); 
    

    /**
     * Brute force method - check each point until collision found or no collision
     */
    void           query(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result) const;
    void           query(const std::vector<trajectory_msgs::JointTrajectoryPoint>& segment, const std::vector<trajectory_msgs::JointTrajectoryPoint>& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const;
    void           query(const std::vector<trajectory_msgs::JointTrajectoryPoint>& segment, const std::vector<trajectory_msgs::JointTrajectoryPoint>& ob_trajectory, const double& traj_start, const double& coll_dist, QueryResult& result) const;


    /**
     * Analytical methods
     */
    void buildTree(const std::vector<ramp_msgs::MotionState>& control_poly, const int& depth, std::vector< std::vector<ramp_msgs::MotionState> >& result) const;
    void deCasteljau(const std::vector<ramp_msgs::MotionState>& control_poly, std::vector< std::vector<ramp_msgs::MotionState> >& result) const;


    void ControlPolyArc(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const ramp_msgs::RampTrajectory& ob_tr, bool& result, std::vector< std::vector<double> >& points_of_collision) const;
    void BezierCircle(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const std::vector<double>& cir_cent, const double& r, bool& result, std::vector< std::vector<double> >& points_of_collision) const;

    void LineNoMotion(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const trajectory_msgs::JointTrajectoryPoint& ob_state, std::vector< std::vector<double> >& points_of_collision) const;
    void BezierNoMotion(const std::vector<ramp_msgs::MotionState>& control_points, const trajectory_msgs::JointTrajectoryPoint& ob_state, std::vector< std::vector<double> >& points_of_collision) const;
    void NoMotionNoMotion(const trajectory_msgs::JointTrajectoryPoint& a, const trajectory_msgs::JointTrajectoryPoint b,  std::vector< std::vector<double> >& points_of_collision) const;


    void LineLineFull
      (const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const;

    void LineLineEndPoints
      (const std::vector<double>& l1_p1, std::vector<double>& l1_p2, const std::vector<double>& l2_p1, std::vector<double>& l2_p2, std::vector< std::vector<double> >& points_of_collision) const;


    // Used by several methods
    void LineArc(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision, bool& result) const;
    
    void LineArc(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const std::vector<trajectory_msgs::JointTrajectoryPoint>& arc_points, std::vector< std::vector<double> >& points_of_collision, bool& result) const;


    void LineCircle(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const std::vector<double>& cir_center, const double& r, bool& result, std::vector< std::vector<double> >& points_of_collision) const;


    void LineArcFull(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const;


    void BezierLineFull(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const;
    void BezierLine(const std::vector<ramp_msgs::MotionState>& control_points, const std::vector<double> l_p1, const std::vector<double> l_p2, std::vector< std::vector<double> >& points_of_collision) const;
 
    void BezierArc(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const;
    

    /**
     * Get circle info for methods involving an arc
     */
    void getCircleInfoTraj(const ramp_msgs::RampTrajectory& traj, double& r, double& h, double& k) const;
    void getCircleInfoPoints(const std::vector<trajectory_msgs::JointTrajectoryPoint>& arc_points, double& r, double& h, double& k) const;



    int findIndexOfCollision(const ramp_msgs::RampTrajectory& trajectory, std::vector<double>& collision_point) const;


    /***** Data Members ****/
    //ramp_msgs::RampTrajectory trajectory_;
    //std::vector<ramp_msgs::RampTrajectory> obstacle_trjs_;
  

    ros::Time t_for;
    ros::Duration d_for;
    ros::Time t_inner_for;
    ros::Duration d_inner_for;
    
    std::vector<ros::Duration> t_ll;
    std::vector<ros::Duration> t_ll_num;


    std::vector<ros::Duration> t_la;
    std::vector<ros::Duration> t_la_num;
    
    std::vector<ros::Duration> t_bl;
    std::vector<ros::Duration> t_ba;
    
    // No obstacle motion
    std::vector<ros::Duration> t_ln;
    std::vector<ros::Duration> t_ln_num;

    std::vector<ros::Duration> t_bn;
    std::vector<ros::Duration> t_bn_num;

    ros::Publisher pub_population;

  private:

    /***** Methods *****/

    /***** Data Members *****/
    Utility                   utility_;
};

#endif
