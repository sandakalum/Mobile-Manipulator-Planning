#ifndef BEZIER_CURVE
#define BEZIER_CURVE

#include "motion_state.h"
#include "utility.h"


class BezierCurve {

public:

  BezierCurve();
  ~BezierCurve();

  void init(const std::vector<MotionState> sp, const double lambda, const MotionState ms_current);
  void init(const ramp_msgs::BezierCurve bi, const MotionState ms_current);
 
 

  const std::vector<MotionState> generateCurve();

  const bool verify();
 

  double A_, B_, C_, D_       ;
  double R_min_               ;
  double t_R_min_             ;
  double l_                   ;
  std::vector<MotionState> segmentPoints_  ;
  std::vector<MotionState> controlPoints_  ;
  std::vector<MotionState> points_         ;
  std::vector<double> u_values_;

  void initControlPoints();
  void initControlPoints(const MotionState start);
  bool print_;

  MotionState ms_init_;
  MotionState ms_max_;
  MotionState ms_begin_;
  MotionState ms_current_;


  double u_0_, u_dot_0_, u_dot_max_, u_target_;
  
  const ramp_msgs::BezierCurve getMsg() const;

private:

  Utility       utility_            ;
  bool          initialized_        ;
  bool          deallocated_        ;
  bool          reachedVMax_        ;

  // Variables to manually track some motion info
  double        x_prev_, y_prev_;
  double        x_dot_prev_, y_dot_prev_;
  double        theta_prev_             ;
  double        theta_dot_prev_         ;
  
  double        MAX_SPEED_;


  void initReflexxes()    ;

  void calculateConstants() ;
  void calculateABCD()      ;
  void calculateT_R_min()     ;
  void calculateR_min()     ;

  const bool finalStateReached() const;

  const MotionState spinOnce();

  void dealloc();


  const bool satisfiesConstraints(const double u_dot, const double u_x, const double u_y) const;
  const double getUDotInitial() const;
  const double getUDotMax(const double u_dot_0) const;
  const double getUDotDotMax(const double u_dot_max) const;

  void printReflexxesInfo() const;

  const MotionState getMS(const double u) const;

  // Approximate initial state of a Bezier curve
  const MotionState getInitialState();
  const double findVelocity(const uint8_t i, const double l, const double slope) const;


};

#endif
