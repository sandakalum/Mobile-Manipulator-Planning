#ifndef MOBILE_ROBOT_H
#define MOBILE_ROBOT_H

#include "ros/ros.h"
#include "utility.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/MotionState.h"
#include "std_msgs/Bool.h"
#include <math.h>

class MobileRobot
{
  public:
    MobileRobot();
  ~MobileRobot();

  /** Methods **/ 
  void init(ros::NodeHandle& h);

  void moveOnTrajectory();
  void moveOnTrajectoryRot(const ramp_msgs::RampTrajectory traj, bool simulation);
  void odomCb(const nav_msgs::Odometry& msg);
  void updateTrajectory(const ramp_msgs::RampTrajectory& msg); 
  void imminentCollisionCb(const std_msgs::Bool msg); 
  void updateCallback(const ros::TimerEvent&);
  void sendTwist(const geometry_msgs::Twist twist) const;
  void controlCycle(geometry_msgs::Twist twist, ros::Time end_time, ros::Rate r);



  /** Data Members **/
  ros::Publisher                    pub_phidget_motor_;
  ros::Publisher                    pub_twist_;
  ros::Publisher                    pub_cmd_vel_;
  ros::Publisher                    pub_update_;
  ros::Subscriber                   sub_odometry_;
  ros::Subscriber                   sub_imminent_collision_;
  ramp_msgs::MotionState            motion_state_; 
  geometry_msgs::Twist              velocity_;
  ramp_msgs::RampTrajectory         trajectory_;
  ros::Timer                        timer_;
  double                            initial_theta_;

  bool                              check_imminent_coll_;
  bool                              imminent_coll_;
  bool                              sim_;

  // static const members
  static const std::string  TOPIC_STR_PHIDGET_MOTOR;
  static const std::string  TOPIC_STR_ODOMETRY;
  static const std::string  TOPIC_STR_UPDATE;
  static const std::string  TOPIC_STR_TWIST;
  static const std::string  TOPIC_STR_IC;
  static const int          ACCELERATION_CONSTANT = 50;
  
  std::vector<ros::Duration> t_points_;
  ros::Time t_prev_traj_;

  private:

  /** Methods **/

  void                        sendTwist() const;
  void                        calculateSpeedsAndTime();
  void                        printVectors() const;
  const bool                  checkImminentCollision();
  const std::vector<double>   computeAcceleration() const;

  void                        accountForAcceleration();


  /** Data Members **/

  Utility                   utility_;
  bool                      restart_;
  int                       num_;
  int                       num_traveled_;
  const unsigned int        k_dof_;
  std::vector<ros::Time>    end_times; 
  std::vector<double>       speeds_linear_;
  std::vector<double>       speeds_angular_;
  std::vector<double>       orientations_;

  geometry_msgs::Twist      twist_;
  geometry_msgs::Twist      zero_twist_;
  ramp_msgs::MotionState    prev_motion_state_; 
  ros::Time                 prev_t_;
  ros::Duration             t_immiColl_;


};

#endif
