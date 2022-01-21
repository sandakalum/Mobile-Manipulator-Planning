#include "bezier_curve.h"


BezierCurve::BezierCurve() : initialized_(false), deallocated_(false), reachedVMax_(false) {}

BezierCurve::~BezierCurve() {}



void BezierCurve::init(const std::vector<MotionState> segment_points, const double lambda, const MotionState ms_current) 
{
  segmentPoints_.clear();
  controlPoints_.clear();
  ms_max_.msg_.velocities.clear();
  ms_max_.msg_.accelerations.clear();

  segmentPoints_ = segment_points;
  l_ = lambda;
  
  MAX_SPEED_ = 1.;

  ms_max_.msg_.velocities.push_back(0.707);
  ms_max_.msg_.velocities.push_back(0.707);
  ms_max_.msg_.velocities.push_back( (2.f*PI)/3.f );
  ms_max_.msg_.accelerations.push_back(0.707);
  ms_max_.msg_.accelerations.push_back(0.707);
  ms_max_.msg_.accelerations.push_back( (2.f*PI)/3.f );
  
  ms_current_ = ms_current;

  //ROS_INFO("Before calling getInitialState");
  ms_init_ = getInitialState();


  /*ROS_INFO("Segment Points:");
  for(int i=0;i<segmentPoints_.size();i++) {
    std::cout<<"\n"<<utility_.toString(segmentPoints_.at(i).msg_);
  }
  std::cout<<"\n";*/

  initControlPoints();
  calculateConstants();

  ms_begin_ = controlPoints_.at(0).msg_;
}





/** Determines if a curve violates angular motion constraints */
const bool BezierCurve::verify() {
  //ROS_INFO("In BezierCurve::verify()");

  double v_max = MAX_SPEED_;
  double w_max = (2.f*PI)/3.f;

  u_dot_0_ = getUDotInitial();

  double u_dot_max = getUDotMax(u_dot_0_);
  if(u_dot_max < 0.0001)
  {
    u_dot_max = getUDotInitial();
  }
  //ROS_INFO("u_dot_max: %f", u_dot_max);

  double x_dot = (A_*t_R_min_ + C_)*u_dot_max;
  double y_dot = (B_*t_R_min_ + D_)*u_dot_max;
  double v_rmin = sqrt(pow(x_dot,2) + pow(y_dot,2));
  double w_rmin = v_rmin / R_min_;
  //ROS_INFO("x_dot: %f y_dot: %f", x_dot, y_dot);
  //ROS_INFO("w_rmin: %f v_rmin: %f R_min: %f t_R_min: %f x_dot: %f y_dot: %f", w_rmin, v_rmin, R_min_, t_R_min_, x_dot, y_dot);
  //ROS_INFO("w_rmin <= w_max: %s", w_rmin <= w_max ? "True" : "False");
  //ROS_INFO("l_: %f", l_);
  

  return ( l_ < 1. && (t_R_min_ >= 0 && t_R_min_ <= 1) && (w_rmin <= w_max) );
}



const double BezierCurve::findVelocity(const uint8_t i, const double l, const double theta) const 
{
  //ROS_INFO("In BezierCurve::findVelocity");
  //ROS_INFO("i: %i l: %f theta: %f", i, l, theta);
  // s = s_0 + v_0*t + 1/2*a*t^2
  // t = (v - v_0) / a;
  
  //ROS_INFO("ms_current_: %s", ms_current_.toString().c_str());
  //ROS_INFO("ms_max_: %s", ms_max_.toString().c_str());

  // Use 2/3 of max acceleration
  double a = (2.*ms_max_.msg_.accelerations.at(i)/3.);

  // Use the current velocity as initial
  double v_0 = ms_current_.msg_.velocities.size() > 0 ?
                ms_current_.msg_.velocities.at(i) : 0;

  double radicand = (2*a*l) + pow(v_0, 2);
  double v = sqrt(radicand);
  
  double v_max = MAX_SPEED_;  
  double v_target = i == 0 ? cos(theta) * v_max : sin(theta) * v_max;


  //ROS_INFO("v_0: %f a: %f radicand: %f v: %f v_max: %f v_target: %f", v_0, a, radicand, v, v_max, v_target);  

   

  // Check for bounds
  if(v > v_target) 
  {
    v = v_target;
  }
  if(v < -v_target) 
  {
    v = -v_target;
  }
   

  // Check for bounds
  /*if(v > ms_max_.msg_.velocities.at(i)) 
  {
    v = ms_max_.msg_.velocities.at(i);
  }
  if(v < -ms_max_.msg_.velocities.at(i)) 
  {
    v = -ms_max_.msg_.velocities.at(i);
  }

  if(slope < 0 && v > 0) 
  {
    v *= -1;
  }*/

  //ROS_INFO("v: %f", v);
  //ROS_INFO("Exiting BezierCurve::findVelocity");
  return v;
} // End findVelocity





const MotionState BezierCurve::getInitialState() {
  //ROS_INFO("In BezierCurve::getInitialState()");

  MotionState result;
  for(uint8_t i=0;i<3;i++) {
    result.msg_.velocities.push_back(0);
  }

  // Find the slope
  double ryse = segmentPoints_.at(1).msg_.positions.at(1) - 
                segmentPoints_.at(0).msg_.positions.at(1);
  double run  = segmentPoints_.at(1).msg_.positions.at(0) - 
                segmentPoints_.at(0).msg_.positions.at(0);
  double slope  = (run != 0) ? ryse / run : ryse;

  double theta = utility_.findAngleFromAToB(segmentPoints_.at(0).msg_.positions, segmentPoints_.at(1).msg_.positions);
  
  // Segment 1 size
  double l = l_ * utility_.positionDistance(
      segmentPoints_.at(0).msg_.positions, 
      segmentPoints_.at(1).msg_.positions);

  
  //ROS_INFO("ryse: %f run: %f slope: %f theta: %f l: %f", ryse, run, slope, theta, l);

  double v_max = 0.33;

  // If change in y is greater
  // no change in x
  // greater change in y, 1st quadrant
  // greater change in y, 3rd quadrant
  // greater change in y, 4th quadrant
  if( (run == 0)                ||
      (slope >= 1)              ||
      (slope == -1 && run < 0)  ||
      (slope < -1) ) 
  {
    result.msg_.velocities.at(1) = findVelocity(1, l, theta);

    if(run == 0.)
    {
      result.msg_.velocities.at(0) = 0.;
    }
    else if(run > 0.)
    {
      result.msg_.velocities.at(0) = sqrt( pow(v_max,2) - pow(result.msg_.velocities.at(1),2) );      
    }
    else
    {
      result.msg_.velocities.at(0) = -sqrt( pow(v_max,2) - pow(result.msg_.velocities.at(1),2) );  
    }
  }
  // if slope == -1 && ryse < 0
  // if slope < 0
  // else
  else {
    result.msg_.velocities.at(0) = findVelocity(0, l, theta);
    if(ryse == 0.)
    {
      result.msg_.velocities.at(1) = 0;
    }
    else if(ryse > 0.)
    {
      result.msg_.velocities.at(1) = sqrt( pow(v_max,2) - pow(result.msg_.velocities.at(0),2) );      
    }
    else
    {
      result.msg_.velocities.at(1) = -sqrt( pow(v_max,2) - pow(result.msg_.velocities.at(0),2) );
    }
  }

  result.msg_.accelerations.push_back(0);
  result.msg_.accelerations.push_back(0);
  

  //ROS_INFO("result: %s", result.toString().c_str());
  //ROS_INFO("Exiting BezierCurve::getInitialState()");
  return result;
} // End getInitialState




/** Returns true if u_dot satisfies the motion constraints 
 *  given a u value - they may be different when testing for u_dot_max */
const bool BezierCurve::satisfiesConstraints(const double u_dot, const double u_x, const double u_y) const {

  double x_dot = ((A_*t_R_min_)+C_)*u_dot;
  double y_dot = ((B_*t_R_min_)+D_)*u_dot;
  double v = sqrt( pow(x_dot,2) + pow(y_dot,2) );

  ////ROS_INFO("x_dot: %f y_dot: %f v: %f", x_dot, y_dot, v);

  if(v > 0.33)
  {
    return false;
  }

  // Square them in case they are negative 
  // Add .0001 because floating-point comparison inaccuracy errors 
  /*if( pow( (A_*u_x+C_)*u_dot,2) > pow((ms_max_.msg_.velocities.at(0))+0.001,2) ||
      pow( (B_*u_y+D_)*u_dot,2) > pow((ms_max_.msg_.velocities.at(1))+0.001,2) )
  {
    return false;
  }*/

  return true;
} // End satisfiesConstraints




const double BezierCurve::getUDotMax(const double u_dot_0) const {
  //std::cout<<"\n\n***** Calculating u_dot_max *****\n";

  // Need the max accelerations
  double x_ddot_max = ms_max_.msg_.accelerations.at(0);
  double y_ddot_max = ms_max_.msg_.accelerations.at(1);
  //ROS_INFO("x_ddot_max: %f y_ddot_max: %f", x_ddot_max, y_ddot_max);

  // Initialize variables
  double u_dot_max;
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 1 : 0;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 1 : 0;

  // New method
  double u_dot_max_x = sqrt( fabs(x_ddot_max / A_) );
  double u_dot_max_y = sqrt( fabs(y_ddot_max / B_) );
  //ROS_INFO("x_ddot_max / A_: %f y_ddot_max / B_: %f", (x_ddot_max / A_), (y_ddot_max / B_));


  //if(print_) {
    //std::cout<<"\nu_x: "<<u_x<<" u_y: "<<u_y;
    //std::cout<<"\nu_dot_max_x: "<<u_dot_max_x<<" u_dot_max_y: "<<u_dot_max_y;
  //}

  // Set a greater and lesser value
  double greater, lesser;
  if(u_dot_max_x > u_dot_max_y) {
    greater = u_dot_max_x;
    lesser = u_dot_max_y;
  }
  else {
    greater = u_dot_max_y;
    lesser = u_dot_max_x;
  }


  /** Set u_dot_max*/

  // If both are zero
  if(u_dot_max_x == 0 && u_dot_max_y == 0) 
  {
    ROS_ERROR("u_dot_max_x == 0 && u_dot_max_y == 0");
    u_dot_max = 0;
  }

  // Test greater
  else if(satisfiesConstraints(greater, u_x, u_y)) 
  {
    //ROS_INFO("Setting u_dot_max to %f", greater);
    u_dot_max = greater;
  }

  // If greater too large, test lesser
  else if(satisfiesConstraints(lesser, u_x, u_y)) 
  {
    //ROS_INFO("Setting u_dot_max to %f", lesser);
    u_dot_max = lesser;    
  }

  // Else, set it to initial u_dot
  else 
  {
    //ROS_INFO("Setting u_dot_max to u_dot_0: %f", u_dot_0);
    u_dot_max = u_dot_0;
  }



  return u_dot_max;
} // End getUDotMax




const double BezierCurve::getUDotInitial() const {
  //ROS_INFO("In BezierCurve::getUDotInitial");
  //std::cout<<"\nms_begin: "<<ms_begin_.toString();
  //std::cout<<"\nms_initVA: "<<ms_init_.toString();
  double x_dot_0 = (ms_begin_.msg_.velocities.size() > 0) ?  ms_begin_.msg_.velocities.at(0) : 
                                                        ms_init_.msg_.velocities.at(0);
  double y_dot_0 = (ms_begin_.msg_.velocities.size() > 0) ?  ms_begin_.msg_.velocities.at(1) : 
                                                        ms_init_.msg_.velocities.at(1);
  
  double u_dot_0_x = fabs(x_dot_0 / (A_*u_0_+C_));
  double u_dot_0_y = fabs(y_dot_0 / (B_*u_0_+D_));
  if(std::isnan(u_dot_0_x)) {
    u_dot_0_x = -9999;
  }
  if(std::isnan(u_dot_0_y)) {
    u_dot_0_y = -9999;
  }
  //std::cout<<"\nx_dot_0: "<<x_dot_0<<" y_dot_0: "<<y_dot_0;
  //std::cout<<"\nu_0: "<<u_0_<<" u_dot_0: "<<u_dot_0_;
  //std::cout<<"\nu_dot_0_x: "<<u_dot_0_x<<" u_dot_0_y: "<<u_dot_0_y;

  // Set a greater and lesser value
  double greater, lesser;
  if(u_dot_0_x > u_dot_0_y) 
  {
    greater = u_dot_0_x;
    lesser = u_dot_0_y;
  }
  else 
  {
    greater = u_dot_0_y;
    lesser = u_dot_0_x;
  }

  // If both are zero
  if(u_dot_0_x == 0 && u_dot_0_y == 0) 
  {
    //ROS_ERROR("u_dot_0_x == 0 && u_dot_0_y == 0");
    return 0;
  }

  // Test greater
  else if(satisfiesConstraints(greater, u_0_, u_0_)) 
  {
    return greater;
  }

  // If greater too large, test lesser
  else if(satisfiesConstraints(lesser, u_0_, u_0_)) 
  {
    return lesser;    
  }

  else 
  {
    //ROS_ERROR("Neither u_dot_0 values satisfy constraints");
    return 0;
  }
} // End getUDotInitial




const double BezierCurve::getUDotDotMax(const double u_dot_max) const 
{
  double result;

  // Set u max acceleration
  // We don't actually use this, but it's necessary for Reflexxes to work
  // Setting u_x and u_y to minimize Au+C or Bu+D - that leads to max a
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 0 : 1;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 0 : 1;
  if(A_*u_x + C_ != 0) 
  {
    result = fabs( (ms_max_.msg_.accelerations.at(0) - A_*u_dot_max) / (A_*u_x+C_) );
  }
  else if (B_*u_y + D_ != 0) 
  {
    result = fabs( (ms_max_.msg_.accelerations.at(1) - B_*u_dot_max) / (B_*u_y+D_) );
  }
  else 
  {
    ROS_ERROR("Neither u acceleration equations are defined!");
    result = 0.1;
  }

  return result;
}






/** Initialize control points 
 *  Sets the first control point and then calls overloaded initControlPoints */
void BezierCurve::initControlPoints() 
{
  //std::cout<<"\nIn initControlPoints 0\n";

  double l_s1 = utility_.positionDistance(segmentPoints_.at(1).msg_.positions, segmentPoints_.at(0).msg_.positions);
  double l_s2 = utility_.positionDistance(segmentPoints_.at(2).msg_.positions, segmentPoints_.at(1).msg_.positions);
  //std::cout<<"\nl_s1: "<<l_s1<<" l_s2: "<<l_s2;

  // If 1st segment's length is smaller than 2nd segment's length
  // Compute first control point and call overloaded method
  if(l_s1 < l_s2) 
  {
    //std::cout<<"\nIn if\n";

    MotionState C0, p0, p1;

    // Set segment points
    p0 = segmentPoints_.at(0);
    p1 = segmentPoints_.at(1);

    // Set orientation of the two segments
    double theta_s1 = utility_.findAngleFromAToB( p0.msg_.positions, 
                                                  p1.msg_.positions);

    /** Positions */
    C0.msg_.positions.push_back( (1-l_)*p0.msg_.positions.at(0) + l_*p1.msg_.positions.at(0) );
    C0.msg_.positions.push_back( (1-l_)*p0.msg_.positions.at(1) + l_*p1.msg_.positions.at(1) );
    C0.msg_.positions.push_back(theta_s1);

    initControlPoints(C0);
  }

  // Else just set all points in here
  else 
  {
    //std::cout<<"\nIn else\n";

    // Adjust l to get control points
    // But keep l_ the same because this block 
    l_ = 1 - l_;

    
    MotionState C0, C1, C2, p0, p1, p2;

    // Set segment points
    p0 = segmentPoints_.at(0);
    p1 = segmentPoints_.at(1);
    p2 = segmentPoints_.at(2);

    // Set orientation of the two segments
    double theta_s1 = utility_.findAngleFromAToB( p0.msg_.positions, 
                                                  p1.msg_.positions);
    double theta_s2 = utility_.findAngleFromAToB( p1.msg_.positions, 
                                                  p2.msg_.positions);

    /** msg_.positions */
    C2.msg_.positions.push_back( (1-l_)*p1.msg_.positions.at(0) + l_*p2.msg_.positions.at(0) );
    C2.msg_.positions.push_back( (1-l_)*p1.msg_.positions.at(1) + l_*p2.msg_.positions.at(1) );
    C2.msg_.positions.push_back(theta_s2);
    
    // Control point 0 is passed in
    // Control Point 1 is the 2nd segment point
    C1 = segmentPoints_.at(1);
    C1.msg_.positions.at(2) = theta_s1;

    // Get x,y msg_.positions of the 3rd control point
    double l_c = utility_.positionDistance(p1.msg_.positions, C2.msg_.positions);
    double x = p1.msg_.positions.at(0) - l_c*cos(theta_s1);
    double y = p1.msg_.positions.at(1) - l_c*sin(theta_s1);


    C0.msg_.positions.push_back(x);  
    C0.msg_.positions.push_back(y);
    C0.msg_.positions.push_back(theta_s1);


    /** C0 Velocities */
    if(C0.msg_.velocities.size() == 0) 
    {
      C0.msg_.velocities.push_back(ms_init_.msg_.velocities.at(0));
      C0.msg_.velocities.push_back(ms_init_.msg_.velocities.at(1));
      C0.msg_.velocities.push_back(0);
    }

    /** C0 Accelerations */
    if(C0.msg_.accelerations.size() == 0) {
      C0.msg_.accelerations.push_back(0);
      C0.msg_.accelerations.push_back(0);
      C0.msg_.accelerations.push_back(0);
    }



    // Push on all the points
    controlPoints_.push_back(C0);
    controlPoints_.push_back(C1);
    controlPoints_.push_back(C2);
    
    /*std::cout<<"\nControl Points:";
    for(int i=0;i<controlPoints_.size();i++) {
      std::cout<<"\n"<<utility_.toString(controlPoints_.at(i).msg_);
    }
    std::cout<<"\n";*/
  } // end else
} // End initControlPoints





/** Initialize the control points of the Bezier curve given the first one */
void BezierCurve::initControlPoints(const MotionState cp_0) {
  //std::cout<<"\nIn initControlPoints 1\n";
  MotionState C0, C1, C2, p0, p1, p2;


  // Set segment points
  p0 = segmentPoints_.at(0);
  p1 = segmentPoints_.at(1);
  p2 = segmentPoints_.at(2);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB( p0.msg_.positions, 
                                                p1.msg_.positions);
  double theta_s2 = utility_.findAngleFromAToB( p1.msg_.positions, 
                                                p2.msg_.positions);

  // Control point 0 is passed in
  // Control Point 1 is the 2nd segment point
  C0 = cp_0;
  C1 = segmentPoints_.at(1);
  C1.msg_.positions.at(2) = theta_s1;

  /** Set 3rd control point */
  // s1 = segment distance between first two control points
  double s1 = sqrt( pow(C1.msg_.positions.at(0) - C0.msg_.positions.at(0), 2) +
                    pow(C1.msg_.positions.at(1) - C0.msg_.positions.at(1), 2) );

  // Get x,y msg_.positions of the 3rd control point
  double x = C1.msg_.positions.at(0) + s1*cos(theta_s2);
  double y = C1.msg_.positions.at(1) + s1*sin(theta_s2);

  // Length of second segment
  double l2 = sqrt( pow(p2.msg_.positions.at(0) - p1.msg_.positions.at(0), 2) +
                    pow(p2.msg_.positions.at(1) - p1.msg_.positions.at(1), 2) );

  // If s1 is greater than entire 2nd segment,
  // set 3rd control point to end of 2nd segment
  if(s1 > l2) 
  {
    C2.msg_.positions.push_back(p2.msg_.positions.at(0));  
    C2.msg_.positions.push_back(p2.msg_.positions.at(1));
  }
  else 
  {
    C2.msg_.positions.push_back(x);  
    C2.msg_.positions.push_back(y);
  }
  C2.msg_.positions.push_back(theta_s2);


  /** C0 Velocities */
  if(C0.msg_.velocities.size() == 0) 
  {
    C0.msg_.velocities.push_back(ms_init_.msg_.velocities.at(0));
    C0.msg_.velocities.push_back(ms_init_.msg_.velocities.at(1));
    C0.msg_.velocities.push_back(0);
  }
  /** C0 Accelerations */
  if(C0.msg_.accelerations.size() == 0) 
  {
    C0.msg_.accelerations.push_back(0);
    C0.msg_.accelerations.push_back(0);
    C0.msg_.accelerations.push_back(0);
  }



  // Push on all the points
  controlPoints_.push_back(C0);
  controlPoints_.push_back(C1);
  controlPoints_.push_back(C2);
  
  /*std::cout<<"\nControl Points:";
  for(int i=0;i<controlPoints_.size();i++) 
  {
    std::cout<<"\n"<<utility_.toString(controlPoints_.at(i).msg_);
  }*/
} // End initControlPoints





void BezierCurve::calculateABCD() 
{
  MotionState p0 = controlPoints_.at(0);
  MotionState p1 = controlPoints_.at(1);
  MotionState p2 = controlPoints_.at(2);

  // A = 2(X0-2X1+X2)
  A_ = 2 * (p0.msg_.positions.at(0) - (2*p1.msg_.positions.at(0)) + p2.msg_.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  B_ = 2 * (p0.msg_.positions.at(1) - (2*p1.msg_.positions.at(1)) + p2.msg_.positions.at(1));

  // C = 2(X1-X0)
  C_ = 2 * (p1.msg_.positions.at(0) - p0.msg_.positions.at(0));

  // D = 2(Y1-Y0)
  D_ = 2 * (p1.msg_.positions.at(1) - p0.msg_.positions.at(1));

  //ROS_INFO("A: %f B: %f C: %f D: %f", A_, B_, C_, D_);
}




/** Calculate the minimum radius along the curve */
void BezierCurve::calculateR_min() 
{
  double numerator_term_one   = ((A_*A_) + (B_*B_)) * (t_R_min_*t_R_min_);
  double numerator_term_two   = 2 * ((A_*C_)+(B_*D_)) * t_R_min_;
  double numerator_term_three = (C_*C_) + (D_*D_);
  double numerator            = pow(numerator_term_one + numerator_term_two + numerator_term_three, 3); 

  double denominator          = pow((B_*C_) - (A_*D_), 2);
 
  R_min_                      = sqrt( numerator / denominator );
  ////ROS_INFO("t_R_min_: %f R_min: %f", t_R_min_, R_min_);
}


/** Calculate time when minimum radius occurs along the curve */
void BezierCurve::calculateT_R_min() 
{
  if(fabs(A_) < 0.000001 && fabs(B_) < 0.000001) 
  {
    ////ROS_INFO("Both A_ and B_ are 0 - setting t_R_min_ to 0");
    t_R_min_ = 0.;
  }
  else 
  {
    double numerator = -((A_*C_) + (B_*D_));
    double denominator = ((A_*A_) + (B_*B_));
    ////ROS_INFO("numerator: %f denominator: %f", numerator, denominator);
    t_R_min_ = numerator / denominator;
  }
}


/** Calculate A,B,C,D, minimum radius, and time of minimum radius */
void BezierCurve::calculateConstants() 
{
  calculateABCD();
  calculateT_R_min();
  calculateR_min();
}



const ramp_msgs::BezierCurve BezierCurve::getMsg() const
{
  ramp_msgs::BezierCurve result;

  for(int i=0;i<segmentPoints_.size();i++)
  {
    result.segmentPoints.push_back(segmentPoints_.at(i).msg_);
  }

  for(int i=0;i<controlPoints_.size();i++)
  {
    result.controlPoints.push_back(controlPoints_.at(i).msg_);
  }

  result.ms_maxVA = ms_max_.msg_;
  result.ms_initialVA = ms_init_.msg_;
  result.l = l_;
  result.ms_begin = ms_begin_.msg_;

  return result;
}
