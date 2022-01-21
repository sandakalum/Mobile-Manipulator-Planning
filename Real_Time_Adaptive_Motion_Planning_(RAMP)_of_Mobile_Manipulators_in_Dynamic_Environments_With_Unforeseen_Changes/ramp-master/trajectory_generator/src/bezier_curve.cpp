#include "bezier_curve.h"



BezierCurve::BezierCurve() : initialized_(false), deallocated_(false), reachedVMax_(false) 
{
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters  = 0;
  reflexxesData_.outputParameters = 0;
  MAX_SPEED = 1.f;
}

BezierCurve::~BezierCurve() 
{
  if(!deallocated_) {
    dealloc(); 
  }
}


void BezierCurve::dealloc() {
  if(!deallocated_) {
    if(reflexxesData_.rml != 0) 
    {
      delete reflexxesData_.rml;
      reflexxesData_.rml = 0;
    }

    if(reflexxesData_.inputParameters != 0) 
    {
      delete reflexxesData_.inputParameters;
      reflexxesData_.inputParameters = 0;
    }
    
    if(reflexxesData_.outputParameters != 0) 
    {
      delete reflexxesData_.outputParameters;
      reflexxesData_.outputParameters = 0;
    }

    deallocated_ = true;
  }
}



void BezierCurve::init(const ramp_msgs::BezierCurve bi, const ramp_msgs::MotionState ms_current) 
{
  segmentPoints_.clear();
  controlPoints_.clear();

  segmentPoints_  = bi.segmentPoints;
  l_              = bi.l;
  theta_prev_     = utility_.findAngleFromAToB(
                                  segmentPoints_.at(0).positions, 
                                  segmentPoints_.at(1).positions);
 
  ms_max_     = bi.ms_maxVA;
  ms_current_ = ms_current;

  if(bi.ms_initialVA.velocities.size() > 0) 
  {
    ms_init_ = bi.ms_initialVA;
  }
  else 
  {
    ms_init_ = getInitialState();
  }

  u_0_      = bi.u_0;
  u_dot_0_  = bi.u_dot_0; 
  u_dot_max_ = bi.u_dot_max;

  if(bi.controlPoints.size() > 0) 
  {
    initControlPoints(bi.controlPoints.at(0));
  }
  else 
  {
    initControlPoints();
  }

  calculateConstants();

  // Set ms_begin
  if(bi.ms_begin.positions.size() > 0) 
  {
    //ROS_INFO("ms_begin passed in: %s", utility_.toString(bi.ms_begin).c_str());
    ms_begin_ = bi.ms_begin;
  }
  else 
  {
    //ROS_INFO("Setting ms_begin to control point 0: %s", utility_.toString(controlPoints_.at(0)).c_str());
    ms_begin_ = controlPoints_.at(0);
  }
  x_prev_         = ms_begin_.positions.at(0);
  y_prev_         = ms_begin_.positions.at(1);
  x_dot_prev_     = ms_begin_.velocities.at(0);
  y_dot_prev_     = ms_begin_.velocities.at(1);
  theta_prev_     = ms_begin_.positions.at(2);
  theta_dot_prev_ = ms_begin_.velocities.at(2);


  // If both C and D == 0, the first two points are the same
  if(fabs(C_) > 0.0001 || fabs(D_) > 0.0001) 
  {
    initReflexxes();
    initialized_ = true;
  } 
}




/** Determines if a curve violates angular motion constraints */
const bool BezierCurve::verify() const 
{
  //////////ROS_INFO("In BezierCurve::verify()");

  double v_max = MAX_SPEED;
  double w_max = (2.f*PI)/3.f;

  double u_dot_max = getUDotMax(u_dot_0_);

  double x_dot  = (A_*t_R_min_ + C_)*u_dot_max;
  double y_dot  = (B_*t_R_min_ + D_)*u_dot_max;
  //double x_dot  = (A_*0. + C_)*u_dot_max;
  //double y_dot  = (B_*0. + D_)*u_dot_max;
  double v_rmin = sqrt(pow(x_dot,2) + pow(y_dot,2));
  double w_rmin = v_rmin / R_min_;
  
  ////////ROS_INFO("u_dot_max: %f", u_dot_max);
  ////////ROS_INFO("x_dot: %f y_dot: %f", x_dot, y_dot);
  ////////ROS_INFO("w_rmin: %f v_rmin: %f R_min: %f t_R_min: %f x_dot: %f y_dot: %f", w_rmin, v_rmin, R_min_, t_R_min_, x_dot, y_dot);
  ////////ROS_INFO("w_rmin <= w_max: %s", w_rmin <= w_max ? "True" : "False");
  ////////ROS_INFO("l_: %f", l_);
  

  return ( l_ < 1. && (t_R_min_ >= 0 && t_R_min_ <= 1) && (w_rmin <= w_max) );
}


void BezierCurve::printReflexxesInfo() const 
{

  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->TargetPositionVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetPositionVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->TargetVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]<<"\n";
} // End printReflexxesInfo




const double BezierCurve::findVelocity(const uint8_t i, const double l, const double theta) const 
{
  //ROS_INFO("In BezierCurve::findVelocity");
  //ROS_INFO("i: %i l: %f theta: %f", i, l, theta);

  // s = s_0 + v_0*t + 1/2*a*t^2
  // t = (v - v_0) / a;
  ////////ROS_INFO("theta: %f", theta);

  // Get quadrant of theta
  int q;
  if(theta >= 0 && theta <= PI/2.)
  {
    q=1;
  }
  else if(theta >= PI/2. && theta <= PI)
  {
    q=2;
  }
  else if(theta <= -PI/2. && theta >= -PI)
  {
    q=3;
  }
  else if(theta <= 0 && theta >= -PI/2.)
  {
    q=4;
  }

  //ROS_INFO("q: %i", q);
  
  // Use 2/3 of max acceleration
  double a = (2.*ms_max_.accelerations.at(i)/3.);

  // Use the current velocity as initial
  double v_0 =  ms_current_.velocities.size() > 0 ?
                ms_current_.velocities.at(i)      : 0;

  double radicand = (2*a*l) + pow(v_0, 2);
  double v = sqrt(radicand);

  if(i == 1 && (q == 3 || q == 4))
  {
    v *= -1;
  }
  if(i == 0 && (q == 2 || q  == 3))
  {
    v *= -1;
  }

  double v_max = sqrt( pow(ms_max_.velocities.at(0),2) + pow(ms_max_.velocities.at(1),2) );
  double v_target_max = i == 0 ? cos(theta) * v_max : sin(theta) * v_max;

  //ROS_INFO("v_0: %f a: %f radicand: %f v: %f v_max: %f v_target_max: %f", v_0, a, radicand, v, v_max, v_target_max);

 

  // Check for bounds
  if(v > v_target_max) 
  {
    v = v_target_max;
  }
  else if(v < 0 && v < v_target_max)
  {
    v = v_target_max;
  }
  else if(v < -v_target_max) 
  {
    v = -v_target_max;
  }
  else if(v > 0 && v > -v_target_max)
  {
    v = -v_target_max;
  }


  //ROS_INFO("v: %f", v);
  return v;
} // End findVelocity





const ramp_msgs::MotionState BezierCurve::getInitialState() 
{
  //ROS_INFO("In BezierCurve::getInitialState()");

  ramp_msgs::MotionState result;
  for(uint8_t i=0;i<3;i++) {
    result.velocities.push_back(0);
  }

  // Find the slope
  double ryse = segmentPoints_.at(1).positions.at(1) - 
                segmentPoints_.at(0).positions.at(1);
  double run  = segmentPoints_.at(1).positions.at(0) - 
                segmentPoints_.at(0).positions.at(0);
  double slope  = (run != 0) ? ryse / run : ryse;

  double theta = utility_.findAngleFromAToB(segmentPoints_.at(0).positions, segmentPoints_.at(1).positions);
  
  
  // Segment 1 size
  double l = l_ * utility_.positionDistance(
      segmentPoints_.at(0).positions, 
      segmentPoints_.at(1).positions);
  
  //ROS_INFO("ryse: %f run: %f slope: %f theta: %f l: %f", ryse, run, slope, theta, l);

  double v_max = MAX_SPEED;

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
    //ROS_INFO("In if");
    result.velocities.at(1) = findVelocity(1, l, theta);

    if(run == 0.)
    {
      result.velocities.at(0) = 0.;
    }
    else if (run > 0.)
    {
      result.velocities.at(0) = sqrt( pow(v_max,2) - pow(result.velocities.at(1),2) );
    }
    else
    {
      result.velocities.at(0) = -sqrt( pow(v_max,2) - pow(result.velocities.at(1),2) );
    }
  }
  // if slope == -1 && ryse < 0
  // if slope < 0
  // else
  else 
  {
    //ROS_INFO("In else");
    result.velocities.at(0) = findVelocity(0, l, theta);
    if(ryse == 0.)
    {
      result.velocities.at(1) = 0;
    }
    else if(ryse > 0.)
    {
      result.velocities.at(1) = sqrt( pow(v_max,2) - pow(result.velocities.at(0),2) );
    }
    else
    {
      result.velocities.at(1) = -sqrt( pow(v_max,2) - pow(result.velocities.at(0),2) );
    }
  }

  //ROS_INFO("Velocities: (%f, %f)", result.velocities.at(0), result.velocities.at(1));

  result.accelerations.push_back(0);
  result.accelerations.push_back(0);
  

  //ROS_INFO("Curve initial state: %s", utility_.toString(result).c_str());
  //ROS_INFO("Exiting BezierCurve::getInitialState");
  return result;
} // End getInitialState




/** Returns true if u_dot satisfies the motion constraints 
 *  given a u value - they may be different when testing for u_dot_max */
const bool BezierCurve::satisfiesConstraints(const double u_dot, const double u_x, const double u_y) const 
{
  ////////////ROS_INFO("In BezierCurve::satisfiesConstraints");
  ////////////ROS_INFO("u_dot: %f u_x: %f u_y: %f", u_dot, u_x, u_y);
  ////////////ROS_INFO("t_R_min_: %f", t_R_min_);
 
  //double x_dot = pow( (A_*u_x+C_)*u_dot,2);
  //double y_dot = pow( (B_*u_y+D_)*u_dot,2);
  double x_dot = ((A_*t_R_min_)+C_)*u_dot;
  double y_dot = ((B_*t_R_min_)+D_)*u_dot;
  double v = sqrt( pow(x_dot,2) + pow(y_dot,2) );

  ////////////ROS_INFO("x_dot: %f y_dot: %f v: %f MAX_SPEED: %f", x_dot, y_dot, v, MAX_SPEED);

  if(v > MAX_SPEED)
  {
    return false;
  }

  // Square them in case they are negative 
  // Add .0001 because floating-point comparison inaccuracy errors 
  /*if( pow( (A_*u_x+C_)*u_dot,2) > pow((ms_max_.velocities.at(0))+0.001,2) ||
      pow( (B_*u_y+D_)*u_dot,2) > pow((ms_max_.velocities.at(1))+0.001,2) )
  {
    return false;
  }*/

  return true;
} // End satisfiesConstraints




const double BezierCurve::getUDotMax(const double u_dot_0) const 
{
  //////////ROS_INFO("In BezierCurve::getUDotMax");
  //////////ROS_INFO("ms_max: %s", utility_.toString(ms_max_).c_str());

  // Need the max accelerations
  double x_ddot_max = ms_max_.accelerations.at(0);
  double y_ddot_max = ms_max_.accelerations.at(1);
  ////////ROS_INFO("x_ddot_max: %f y_ddot_max: %f", x_ddot_max, y_ddot_max);

  // New method
  double u_dot_max_x = sqrt( fabs(x_ddot_max / A_) );
  double u_dot_max_y = sqrt( fabs(y_ddot_max / B_) );
  ////////ROS_INFO("x_ddot_max: %f A_: %f x_ddot_max / A_: %f y_ddot_max: %f B_: %f y_ddot_max / B_: %f", x_ddot_max, A_, (x_ddot_max / A_), y_ddot_max, B_, (y_ddot_max / B_));


  /*//if(print_) {
    std::cout<<"\nu_x: "<<u_x<<" u_y: "<<u_y;
    std::cout<<"\nu_dot_max_x: "<<u_dot_max_x<<" u_dot_max_y: "<<u_dot_max_y;
  //}*/

  // Set a greater and lesser value
  double greater, lesser;
  if(u_dot_max_x > u_dot_max_y) 
  {
    greater = u_dot_max_x;
    lesser  = u_dot_max_y;
  }
  else 
  {
    greater = u_dot_max_y;
    lesser  = u_dot_max_x;
  }


  /** Set u_dot_max*/
  // Initialize variables
  double u_dot_max;
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 1 : 0;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 1 : 0;

  // If both are zero
  if(u_dot_max_x == 0 && u_dot_max_y == 0) 
  {
    ////ROS_ERROR("u_dot_max_x == 0 && u_dot_max_y == 0");
    u_dot_max = 0;
  }

  // Test greater
  else if(satisfiesConstraints(greater, u_x, u_y)) 
  {
    ////////ROS_INFO("Setting u_dot_max to %f", greater);
    u_dot_max = greater;
  }

  // If greater too large, test lesser
  else if(satisfiesConstraints(lesser, u_x, u_y)) 
  {
    ////////ROS_INFO("Setting u_dot_max to %f", lesser);
    u_dot_max = lesser;    
  }

  // Else, set it to initial u_dot
  else 
  {
    ////////ROS_INFO("Setting u_dot_max to u_dot_0: %f", u_dot_0);
    u_dot_max = u_dot_0;
  }



  return u_dot_max;
} // End getUDotMax




const double BezierCurve::getUDotInitial() const 
{
  ////////ROS_INFO("In BezierCurve::getUDotInitial");

  double x_dot_0 = (ms_begin_.velocities.size() > 0) ?  ms_begin_.velocities.at(0) : 
                                                        ms_init_.velocities.at(0);
  double y_dot_0 = (ms_begin_.velocities.size() > 0) ?  ms_begin_.velocities.at(1) : 
                                                        ms_init_.velocities.at(1);
  
  ////////ROS_INFO("ms_begin: %s", utility_.toString(ms_begin_).c_str());
  ////////ROS_INFO("ms_init: %s", utility_.toString(ms_init_).c_str());
  
  double u_dot_0_x = fabs(x_dot_0 / (A_*u_0_+C_));
  double u_dot_0_y = fabs(y_dot_0 / (B_*u_0_+D_));
  if(isnan(u_dot_0_x)) 
  {
    u_dot_0_x = -9999;
  }
  if(isnan(u_dot_0_y)) 
  {
    u_dot_0_y = -9999;
  }


  ////////ROS_INFO("x_dot_0: %f y_dot_0: %f u_dot_0_x: %f u_dot_0_y: %f", x_dot_0, y_dot_0, u_dot_0_x, u_dot_0_y);


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
    ////ROS_ERROR("u_dot_0_x == 0 && u_dot_0_y == 0");
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
    ////ROS_ERROR("Neither u_dot_0 values satisfy constraints");
    return 0;
  }
} // End getUDotInitial




const double BezierCurve::getUDotDotMax(const double u_dot_max) const {
  double result;

  // Set u max acceleration
  // We don't actually use this, but it's necessary for Reflexxes to work
  // Setting u_x and u_y to minimize Au+C or Bu+D - that leads to max a
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 0 : 1;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 0 : 1;
  if(A_*u_x + C_ != 0) {
    result = fabs( (ms_max_.accelerations.at(0) - A_*u_dot_max) / (A_*u_x+C_) );
  }
  else if (B_*u_y + D_ != 0) {
    result = fabs( (ms_max_.accelerations.at(1) - B_*u_dot_max) / (B_*u_y+D_) );
  }
  else {
    ////ROS_ERROR("Neither u acceleration equations are defined!");
    result = 0.1;
  }

  return result;
}



/** This method initializes the necessary Reflexxes variables */
void BezierCurve::initReflexxes() 
{
  ////////ROS_INFO("In BezierCurve::initReflexxes");

  // Set some variables for readability
  double x_dot_0        = ms_begin_.velocities.at(0);
  double y_dot_0        = ms_begin_.velocities.at(1);
  double x_dot_max      = ms_max_.velocities.at(0);
  double y_dot_max      = ms_max_.velocities.at(1);
  double x_dot_dot_max  = ms_max_.accelerations.at(0);
  double y_dot_dot_max  = ms_max_.accelerations.at(1);

  // Initialize Reflexxes variables
  reflexxesData_.rml              = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters  = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;


  // Get initial and maximum velocity of Bezier parameter 
  // TODO: Choose between local vs. global variable
  double u_dot_0 = getUDotInitial(); 
  if(u_dot_0_ != 0) 
  {
    u_dot_0 = u_dot_0_;
  }
  else 
  {
    u_dot_0_ = u_dot_0;
  }

  double u_dot_max = getUDotMax(u_dot_0);
  if(u_dot_max_ != 0)
  {
    u_dot_max  = u_dot_max_;
  }
  else
  {
    u_dot_max_ = u_dot_max;
  }
  //////////ROS_INFO("u_dot_max: %f u_dot_max_: %f", u_dot_max, u_dot_max_);


  // Set the position and velocity Reflexxes variables
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]     = u_0_;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]     = u_0_ > 0 ? u_dot_max_ : u_dot_0_;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]         = u_dot_max;

  // Set u max acceleration
  // We don't actually use this, but it's necessary for Reflexxes to work
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 
    getUDotDotMax(u_dot_max);


  /*
   * THIS PART CAN LEAD TO BAD TRAJECTORIES
   * If num_uDotMax is 1 too short, robot has to stop and rotate at the end of curve
   * If num_uDotMax is 1 too large, robot has to loop back around 
   */

  // # of u_dot_max's to reach 1
  // Round it off the nearest tenth
  double num_uDotMax = 1./u_dot_max;

  //////ROS_INFO("u_dot_max: %f", u_dot_max);
  //////ROS_INFO("num_uDotMax: %f", num_uDotMax);
  
  // # of cycles to reach 1
  // num_cycles should be an int, but C++ was giving me the incorrect value as an int
  // e.g. 1.9 / 0.1 = 18 instead of 19
  float num_cycles = (int)(num_uDotMax / CYCLE_TIME_IN_SECONDS);

  // Position if moving at max velocity
  double p_maxv = u_dot_max*CYCLE_TIME_IN_SECONDS * num_cycles;

  //////ROS_INFO("p_maxv: %f p_maxv+(u_dot_max/10.f): %f", p_maxv, (p_maxv+(u_dot_max/10.f)));

  if(p_maxv + (u_dot_max / 10.f) < 1.0001)
  {
    //////ROS_INFO("Undershooting: Adding another cycle");
    //////ROS_INFO("p_maxv before: %f p_maxv after: %f", p_maxv, (p_maxv+(u_dot_max/10.f)));
    p_maxv += (u_dot_max/10.f);
  }
 

  //////ROS_INFO("num_udotmax: %f num_cycles: %f p_maxv: %f", num_uDotMax, num_cycles, p_maxv);
 
 
  // Set targets
  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = p_maxv;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = 
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
 

  //if(print_) {
    //printReflexxesInfo();
  //}


  reflexxesData_.resultValue = 0;
  ////////ROS_INFO("Exiting BezierCurve::initReflexxes");
} // End initReflexxes





/** Initialize control points 
 *  Sets the first control point and then calls overloaded initControlPoints */
void BezierCurve::initControlPoints() 
{
  ////////ROS_INFO("In BezierCurve::initControlPoints()");

  double l_s1 = utility_.positionDistance(segmentPoints_.at(1).positions, segmentPoints_.at(0).positions);
  double l_s2 = utility_.positionDistance(segmentPoints_.at(2).positions, segmentPoints_.at(1).positions);
  ////////ROS_INFO("l_s1: %f l_s2: %f", l_s1, l_s2);

  // If 1st segment's length is smaller than 2nd segment's length
  // Compute first control point and call overloaded method
  if(l_s1 < l_s2) 
  {
    //std::cout<<"\nIn if\n";

    ramp_msgs::MotionState C0, p0, p1;

    // Set segment points
    p0 = segmentPoints_.at(0);
    p1 = segmentPoints_.at(1);

    // Set orientation of the two segments
    double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                  p1.positions);

    /** Positions */
    C0.positions.push_back( (1-l_)*p0.positions.at(0) + l_*p1.positions.at(0) );
    C0.positions.push_back( (1-l_)*p0.positions.at(1) + l_*p1.positions.at(1) );
    C0.positions.push_back(theta_s1);

    initControlPoints(C0);
  }

  // Else just set all points in here
  else 
  {
    //std::cout<<"\nIn else\n";

    // Adjust l to get control points
    // But keep l_ the same because this block 
    l_ = 1 - l_;

    
    ramp_msgs::MotionState C0, C1, C2, p0, p1, p2;

    // Set segment points
    p0 = segmentPoints_.at(0);
    p1 = segmentPoints_.at(1);
    p2 = segmentPoints_.at(2);

    // Set orientation of the two segments
    double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                  p1.positions);
    double theta_s2 = utility_.findAngleFromAToB( p1.positions, 
                                                  p2.positions);

    /** Positions */
    C2.positions.push_back( (1-l_)*p1.positions.at(0) + l_*p2.positions.at(0) );
    C2.positions.push_back( (1-l_)*p1.positions.at(1) + l_*p2.positions.at(1) );
    C2.positions.push_back(theta_s2);
    
    // Control point 0 is passed in
    // Control Point 1 is the 2nd segment point
    C1 = segmentPoints_.at(1);
    C1.positions.at(2) = theta_s1;

    // Get x,y positions of the 3rd control point
    double l_c = utility_.positionDistance(p1.positions, C2.positions);
    double x = p1.positions.at(0) - l_c*cos(theta_s1);
    double y = p1.positions.at(1) - l_c*sin(theta_s1);


    C0.positions.push_back(x);  
    C0.positions.push_back(y);
    C0.positions.push_back(theta_s1);


    /** C0 Velocities */
    if(C0.velocities.size() == 0) 
    {
      ////////ROS_INFO("Setting velocities from ms_init_");
      ////////ROS_INFO("ms_init: %s", utility_.toString(ms_init_).c_str());
      C0.velocities.push_back(ms_init_.velocities.at(0));
      C0.velocities.push_back(ms_init_.velocities.at(1));
      C0.velocities.push_back(0);
    }

    /** C0 Accelerations */
    if(C0.accelerations.size() == 0) 
    {
      C0.accelerations.push_back(0);
      C0.accelerations.push_back(0);
      C0.accelerations.push_back(0);
    }



    // Push on all the points
    controlPoints_.push_back(C0);
    controlPoints_.push_back(C1);
    controlPoints_.push_back(C2);
    
    /*std::cout<<"\nControl Points:";
    for(int i=0;i<controlPoints_.size();i++) {
      std::cout<<"\n"<<utility_.toString(controlPoints_.at(i));
    }
    std::cout<<"\n";*/
  } // end else
} // End initControlPoints





/** Initialize the control points of the Bezier curve given the first one */
void BezierCurve::initControlPoints(const ramp_msgs::MotionState cp_0) 
{
  ////////ROS_INFO("In BezierCurve::initControlPoints(ramp_msgs::MotionState)");
  ramp_msgs::MotionState C0, C1, C2, p0, p1, p2;


  // Set segment points
  p0 = segmentPoints_.at(0);
  p1 = segmentPoints_.at(1);
  p2 = segmentPoints_.at(2);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                p1.positions);
  double theta_s2 = utility_.findAngleFromAToB( p1.positions, 
                                                p2.positions);

  ////////ROS_INFO("theta_s1: %f theta_s2: %f", theta_s1, theta_s2);

  // Control point 0 is passed in
  // Control Point 1 is the 2nd segment point
  C0 = cp_0;
  C1 = segmentPoints_.at(1);
  C1.positions.at(2) = theta_s1;

  /** Set 3rd control point */
  // s1 = segment distance between first two control points
  double s1 = sqrt( pow(C1.positions.at(0) - C0.positions.at(0), 2) +
                    pow(C1.positions.at(1) - C0.positions.at(1), 2) );

  // Get x,y positions of the 3rd control point
  double x = C1.positions.at(0) + s1*cos(theta_s2);
  double y = C1.positions.at(1) + s1*sin(theta_s2);

  // Length of second segment
  double l2 = sqrt( pow(p2.positions.at(0) - p1.positions.at(0), 2) +
                    pow(p2.positions.at(1) - p1.positions.at(1), 2) );

  // If s1 is greater than entire 2nd segment,
  // set 3rd control point to end of 2nd segment
  if(s1 > l2) {
    C2.positions.push_back(p2.positions.at(0));  
    C2.positions.push_back(p2.positions.at(1));
  }
  else {
    C2.positions.push_back(x);  
    C2.positions.push_back(y);
  }
  C2.positions.push_back(theta_s2);


  /** C0 Velocities */
  if(C0.velocities.size() == 0) 
  {
    ////////////ROS_INFO("Setting velocities from ms_init");
    C0.velocities.push_back(ms_init_.velocities.at(0));
    C0.velocities.push_back(ms_init_.velocities.at(1));
    C0.velocities.push_back(0);
  }
  /** C0 Accelerations */
  if(C0.accelerations.size() == 0) {
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
  }



  // Push on all the points
  controlPoints_.push_back(C0);
  controlPoints_.push_back(C1);
  controlPoints_.push_back(C2);
  
  /*std::cout<<"\nControl Points:";
  for(int i=0;i<controlPoints_.size();i++) 
  {
    std::cout<<"\n"<<utility_.toString(controlPoints_.at(i));
  }*/
} // End initControlPoints





/** Returns true when Reflexxes has reached its targets */
const bool BezierCurve::finalStateReached() const 
{
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}


void BezierCurve::calculateABCD() 
{
  ramp_msgs::MotionState p0 = controlPoints_.at(0);
  ramp_msgs::MotionState p1 = controlPoints_.at(1);
  ramp_msgs::MotionState p2 = controlPoints_.at(2);

  // A = 2(X0-2X1+X2)
  A_ = 2 * (p0.positions.at(0) - (2*p1.positions.at(0)) + p2.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  B_ = 2 * (p0.positions.at(1) - (2*p1.positions.at(1)) + p2.positions.at(1));

  // C = 2(X1-X0)
  C_ = 2 * (p1.positions.at(0) - p0.positions.at(0));

  // D = 2(Y1-Y0)
  D_ = 2 * (p1.positions.at(1) - p0.positions.at(1));

  ////////////ROS_INFO("A: %f B: %f C: %f D: %f", A_, B_, C_, D_);
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
  ////////////ROS_INFO("t_R_min_: %f R_min: %f", t_R_min_, R_min_);
}


/** Calculate time when minimum radius occurs along the curve */
void BezierCurve::calculateT_R_min() 
{
  if(fabs(A_) < 0.000001 && fabs(B_) < 0.000001) 
  {
    ////////////ROS_INFO("Both A_ and B_ are 0 - setting t_R_min_ to 0");
    t_R_min_ = 0.;
  }
  else 
  {
    double numerator = -((A_*C_) + (B_*D_));
    double denominator = ((A_*A_) + (B_*B_));
    ////////////ROS_INFO("numerator: %f denominator: %f", numerator, denominator);
    t_R_min_ = numerator / denominator;
  }
}


/** Calculate A,B,C,D, minimum radius, and time of minimum radius */
void BezierCurve::calculateConstants() {
  calculateABCD();
  calculateT_R_min();
  calculateR_min();
}



/** Generate all the motion states on the curve */
const std::vector<ramp_msgs::MotionState> BezierCurve::generateCurve() 
{
  ////////ROS_INFO("Entered BezierCurve::generateCurve()");
  ////////ROS_INFO("Points so far: ");
  for(uint8_t i=0;i<points_.size();i++)
  {
    ////////ROS_INFO("Point %i, %s", i, utility_.toString(points_.at(i)).c_str());
  }
  //printReflexxesInfo();

  

  /*std::cout<<"\nControl Points:";
  for(int i=0;i<controlPoints_.size();i++) {
    std::cout<<"\n"<<utility_.toString(controlPoints_.at(i));
  }
  std::cout<<"\n";*/


  if(initialized_) 
  {

    reflexxesData_.resultValue = 0;
   
    // Push on beginning point
    points_.push_back(ms_begin_);
    u_values_.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]);

    while(!finalStateReached()) 
    {
      points_.push_back(spinOnce());
    }

    // Set u_target
    u_target_ = reflexxesData_.inputParameters->TargetPositionVector->VecData[0];
    dealloc();
  }

  

  //////ROS_INFO("After generating curve, u_values:");
  for(int i=0;i<u_values_.size();i++)
  {
    //////ROS_INFO("u_values_[%i]: %f", i, u_values_.at(i));
  }

  //////////ROS_INFO("Exiting BezierCurve::generateCurve()");
  return points_;
} // End generateCurve


void BezierCurve::generateCurveOOP()
{
  ////////ROS_INFO("Entered BezierCurve::generateCurve()");
  ////////ROS_INFO("Points so far: ");
  for(uint8_t i=0;i<points_.size();i++)
  {
    //////ROS_INFO("Point %i, %s", i, utility_.toString(points_.at(i)).c_str());
  }
  //printReflexxesInfo();
  

  /*std::cout<<"\nControl Points:";
  for(int i=0;i<controlPoints_.size();i++) {
    std::cout<<"\n"<<utility_.toString(controlPoints_.at(i));
  }
  std::cout<<"\n";*/

  if(initialized_) {

    reflexxesData_.resultValue = 0;
   
    // Push on beginning point
    points_.push_back(ms_begin_);
    u_values_.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]);

    while(!finalStateReached()) 
    {
      points_.push_back(spinOnce());
    }

    // Set u_target
    u_target_ = reflexxesData_.inputParameters->TargetPositionVector->VecData[0];
    dealloc();
  }

  

  /*////////////ROS_INFO("After generating curve, u_values:");
  for(int i=0;i<u_values_.size();i++)
  {
    ////////////ROS_INFO("u_values_[%i]: %f", i, u_values_.at(i));
  }*/

  //////////ROS_INFO("Exiting BezierCurve::generateCurve()");
}


// TODO: Make const
const ramp_msgs::MotionState BezierCurve::buildMotionState(const ReflexxesData data) 
{
  ////////ROS_INFO("In BezierCurve::buildMotionState");
  ramp_msgs::MotionState result;

  // Set variables to make equations more readable
  double u          = reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  double u_dot      = reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  double u_dot_dot  = reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  double X0         = controlPoints_.at(0).positions.at(0);
  double X1         = controlPoints_.at(1).positions.at(0);
  double X2         = controlPoints_.at(2).positions.at(0);
  double Y0         = controlPoints_.at(0).positions.at(1);
  double Y1         = controlPoints_.at(1).positions.at(1);
  double Y2         = controlPoints_.at(2).positions.at(1);
  
  u_values_.push_back(u);

  /** Create new point */
  // Position
  double x      = (pow((1-u),2) * X0) + ((2*u)*(1-u)*X1) + (pow(u,2)*X2);
  double y      = (pow((1-u),2) * Y0) + ((2*u)*(1-u)*Y1) + (pow(u,2)*Y2);
  double theta  = utility_.findAngleFromAToB(x_prev_, y_prev_, x, y);
  ////////ROS_INFO("x_prev_: %f x: %f y_prev_: %f y: %f", x_prev_, x, y_prev_, y); 

  
  // Velocity
  double x_dot = ((A_*u) + C_)*u_dot;
  double y_dot = (x_dot*(B_*u+D_)) / (A_*u+C_);
  ////////ROS_INFO("theta_prev: %f", theta_prev_);
  ////////ROS_INFO("utility_.findDistanceBetweenAngles(theta_prev_, theta): %f", utility_.findDistanceBetweenAngles(theta_prev_, theta));
  double theta_dot      = utility_.findDistanceBetweenAngles(theta_prev_, theta) / CYCLE_TIME_IN_SECONDS;

  // Acceleration
  double  x_dot_dot = u_dot_dot*(A_*u+C_) + A_*u_dot*u_dot;
  double  y_dot_dot = u_dot_dot*(B_*u+D_) + B_*u_dot*u_dot;
  ////////ROS_INFO("theta_dot_prev: %f", theta_dot_prev_);
  ////////ROS_INFO("utility_.findDistanceBetweenAngles(theta_dot_prev_, theta_dot): %f", utility_.findDistanceBetweenAngles(theta_dot_prev_, theta_dot));
  double theta_dot_dot  = utility_.findDistanceBetweenAngles(theta_dot, theta_dot_prev_) / CYCLE_TIME_IN_SECONDS;


  // Set previous motion values 
  x_prev_         = x;
  y_prev_         = y;
  x_dot_prev_     = x_dot;
  y_dot_prev_     = y_dot;
  theta_prev_     = theta;
  theta_dot_prev_ = theta_dot;
  
  if(print_) 
  {
    //printf("\n");
    ////////ROS_INFO("u: %f u_dot: %f u_dot_dot: %f", u, u_dot, u_dot_dot);
    ////////ROS_INFO("x: %f             y: %f", x, y);
    ////////ROS_INFO("x_dot: %f         y_dot: %f       theta_dot: %f", x_dot, y_dot, theta_dot);
    ////////ROS_INFO("x_dot_dot: %f     y_dot_dot: %f       theta_dot_dot: %f", x_dot_dot, y_dot_dot, theta_dot_dot);
  }

  // Push values onto MotionState
  result.positions.push_back(x);
  result.positions.push_back(y);
  result.positions.push_back(theta);

  result.velocities.push_back(x_dot);
  result.velocities.push_back(y_dot);
  result.velocities.push_back(theta_dot);

  result.accelerations.push_back(x_dot_dot);
  result.accelerations.push_back(y_dot_dot);
  result.accelerations.push_back(theta_dot_dot);

  return result;
}


void BezierCurve::buildMotionStateOOP(const ReflexxesData& data, ramp_msgs::MotionState& result)
{
  // Set variables to make equations more readable
  double u          = reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  double u_dot      = reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  double u_dot_dot  = reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  double X0         = controlPoints_.at(0).positions.at(0);
  double X1         = controlPoints_.at(1).positions.at(0);
  double X2         = controlPoints_.at(2).positions.at(0);
  double Y0         = controlPoints_.at(0).positions.at(1);
  double Y1         = controlPoints_.at(1).positions.at(1);
  double Y2         = controlPoints_.at(2).positions.at(1);
  
  u_values_.push_back(u);

  /** Create new point */
  // Position
  double x      = (pow((1-u),2) * X0) + ((2*u)*(1-u)*X1) + (pow(u,2)*X2);
  double y      = (pow((1-u),2) * Y0) + ((2*u)*(1-u)*Y1) + (pow(u,2)*Y2);
  double theta  = utility_.findAngleFromAToB(x_prev_, y_prev_, x, y);
  ////////ROS_INFO("x_prev_: %f x: %f y_prev_: %f y: %f", x_prev_, x, y_prev_, y); 

  
  // Velocity
  double x_dot = ((A_*u) + C_)*u_dot;
  double y_dot = (x_dot*(B_*u+D_)) / (A_*u+C_);
  ////////ROS_INFO("theta_prev: %f", theta_prev_);
  ////////ROS_INFO("utility_.findDistanceBetweenAngles(theta_prev_, theta): %f", utility_.findDistanceBetweenAngles(theta_prev_, theta));
  double theta_dot      = utility_.findDistanceBetweenAngles(theta_prev_, theta) / CYCLE_TIME_IN_SECONDS;

  // Acceleration
  double  x_dot_dot = u_dot_dot*(A_*u+C_) + A_*u_dot*u_dot;
  double  y_dot_dot = u_dot_dot*(B_*u+D_) + B_*u_dot*u_dot;
  ////////ROS_INFO("theta_dot_prev: %f", theta_dot_prev_);
  ////////ROS_INFO("utility_.findDistanceBetweenAngles(theta_dot_prev_, theta_dot): %f", utility_.findDistanceBetweenAngles(theta_dot_prev_, theta_dot));
  double theta_dot_dot  = utility_.findDistanceBetweenAngles(theta_dot, theta_dot_prev_) / CYCLE_TIME_IN_SECONDS;


  // Set previous motion values 
  x_prev_         = x;
  y_prev_         = y;
  x_dot_prev_     = x_dot;
  y_dot_prev_     = y_dot;
  theta_prev_     = theta;
  theta_dot_prev_ = theta_dot;
  
  if(print_) 
  {
    //printf("\n");
    ////////ROS_INFO("u: %f u_dot: %f u_dot_dot: %f", u, u_dot, u_dot_dot);
    ////////ROS_INFO("x: %f             y: %f", x, y);
    ////////ROS_INFO("x_dot: %f         y_dot: %f       theta_dot: %f", x_dot, y_dot, theta_dot);
    ////////ROS_INFO("x_dot_dot: %f     y_dot_dot: %f       theta_dot_dot: %f", x_dot_dot, y_dot_dot, theta_dot_dot);
  }

  // Push values onto MotionState
  result.positions.push_back(x);
  result.positions.push_back(y);
  result.positions.push_back(theta);

  result.velocities.push_back(x_dot);
  result.velocities.push_back(y_dot);
  result.velocities.push_back(theta_dot);

  result.accelerations.push_back(x_dot_dot);
  result.accelerations.push_back(y_dot_dot);
  result.accelerations.push_back(theta_dot_dot);
}




/** Call Reflexxes once and return the next motion state */
// TODO: Clean up?
const ramp_msgs::MotionState BezierCurve::spinOnce() 
{
  //////ROS_INFO("In BezierCurve::spinOnce()");
  ramp_msgs::MotionState result;


  // Call Reflexxes
  reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition( 
                                 *reflexxesData_.inputParameters, 
                                  reflexxesData_.outputParameters, 
                                  reflexxesData_.flags );
  //////////////ROS_INFO("resultValue: %i", reflexxesData_.resultValue);
  
  // Check if the max velocity has been reached
  // If not, adjust the target position based on how far we've moved
  if(!reachedVMax_)
  {
    //////ROS_INFO("Adjusting target position");
    double a = reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] * 
                  CYCLE_TIME_IN_SECONDS;
    double b = reflexxesData_.outputParameters->NewPositionVector->VecData[0] - 
                  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];

    //////ROS_INFO("a: %f b: %f a-b: %f", a, b, a-b);
    //////ROS_INFO("Current Target: %f", reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);


    // Adjust target position
    reflexxesData_.inputParameters->TargetPositionVector->VecData[0] -= (a-b);
    //////ROS_INFO("New Target: %f", reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);
    
    if(reflexxesData_.inputParameters->TargetPositionVector->VecData[0] < 
        reflexxesData_.inputParameters->CurrentPositionVector->VecData[0])
    {
      ////ROS_ERROR("Adjustment to target position makes target less than the current position");
      ////ROS_ERROR("Current u: %f Target u: %f", reflexxesData_.inputParameters->CurrentPositionVector->VecData[0], reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);
      ////ROS_ERROR("Setting resultValue=1 to stop planning");
      reflexxesData_.resultValue = 1;
    }
    else 
    {
      // Check the new velocity to see if we're at the target
      reachedVMax_ = (fabs(reflexxesData_.outputParameters->NewVelocityVector->VecData[0] - 
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]) < 0.0001);
      
      if(reachedVMax_) {
        reflexxesData_.inputParameters->TargetPositionVector->VecData[0] -= 0.000001;
      }
      
      // Call Reflexxes after adjusting in case we are currently at the new target
      reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition( 
                                   *reflexxesData_.inputParameters, 
                                    reflexxesData_.outputParameters, 
                                    reflexxesData_.flags );
    } // end else no problem with adjustment
  } // end if adjusting target position

  // Build the new motion state
  //result = buildMotionState(reflexxesData_);
  buildMotionStateOOP(reflexxesData_, result);

  // Set current vectors to the output 
  *reflexxesData_.inputParameters->CurrentPositionVector = 
    *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
    *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
    *reflexxesData_.outputParameters->NewAccelerationVector;

  //////ROS_INFO("Exiting BezierCurve::spinOnce()");
  return result;
} // End spinOnce
