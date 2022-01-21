#ifndef BEZIER_CURVE
#define BEZIER_CURVE

#include "utility.h"


class BezierCurve {

public:

  BezierCurve();
  ~BezierCurve();

  void init(const ramp_msgs::BezierCurve bi, const ramp_msgs::MotionState ms_current);
 
 

  const std::vector<ramp_msgs::MotionState> generateCurve();
  void generateCurveOOP();

  const bool verify() const;
 

  double A_, B_, C_, D_       ;
  double R_min_               ;
  double t_R_min_             ;
  double l_                   ;
  std::vector<ramp_msgs::MotionState> segmentPoints_  ;
  std::vector<ramp_msgs::MotionState> controlPoints_  ;
  std::vector<ramp_msgs::MotionState> points_         ;
  std::vector<double> u_values_;

  void initControlPoints();
  void initControlPoints(const ramp_msgs::MotionState start);
  bool print_;

  ramp_msgs::MotionState ms_init_;
  ramp_msgs::MotionState ms_max_;
  ramp_msgs::MotionState ms_begin_;
  ramp_msgs::MotionState ms_current_;


  double u_0_, u_dot_0_, u_dot_max_, u_target_;
private:

  Utility       utility_            ;
  ReflexxesData reflexxesData_      ;
  bool          initialized_        ;
  bool          deallocated_        ;
  bool          reachedVMax_        ;

  // Variables to manually track some motion info
  double        x_prev_, y_prev_;
  double        x_dot_prev_, y_dot_prev_;
  double        theta_prev_             ;
  double        theta_dot_prev_         ;


  double MAX_SPEED;


  void initReflexxes()    ;

  void calculateConstants() ;
  void calculateABCD()      ;
  void calculateT_R_min()     ;
  void calculateR_min()     ;

  const bool finalStateReached() const;

  const ramp_msgs::MotionState spinOnce();

  void dealloc();


  const bool satisfiesConstraints(const double u_dot, const double u_x, const double u_y) const;
  const double getUDotInitial() const;
  const double getUDotMax(const double u_dot_0) const;
  const double getUDotDotMax(const double u_dot_max) const;

  void printReflexxesInfo() const;

  const ramp_msgs::MotionState getMS(const double u) const;

  // Approximate initial state of a Bezier curve
  const ramp_msgs::MotionState getInitialState();
  const double findVelocity(const uint8_t i, const double l, const double slope) const;


  // TODO: Make const
  const ramp_msgs::MotionState buildMotionState(const ReflexxesData data);
  void buildMotionStateOOP(const ReflexxesData& data, ramp_msgs::MotionState& result);

  const ReflexxesData adjustTargets(const ReflexxesData data) const;
};

#endif
