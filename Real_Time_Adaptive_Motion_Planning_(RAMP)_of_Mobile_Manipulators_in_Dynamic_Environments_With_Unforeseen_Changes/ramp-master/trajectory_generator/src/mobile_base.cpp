#include "mobile_base.h"

/** Constructor */
MobileBase::MobileBase() : planning_full_(false), i_XDOF_(0), i_THETADOF_(1) 
{
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
  MAX_SPEED = 1.f;
} 


/** Destructor */
MobileBase::~MobileBase() 
{
  if(reflexxesData_.rml != 0) 
  {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }
  if(reflexxesData_.inputParameters) 
  {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  if(reflexxesData_.outputParameters != 0) 
  {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}



/** Initialize Reflexxes variables */
void MobileBase::initReflexxes() 
{

  // Set DOF
  reflexxesData_.NUMBER_OF_DOFS = 2;

  // Initialize all relevant objects of the Type II Reflexxes Motion Library
  if(reflexxesData_.rml == 0) 
  {
    reflexxesData_.rml = new ReflexxesAPI( 
            reflexxesData_.NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );

    reflexxesData_.inputParameters = new RMLPositionInputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );

    reflexxesData_.outputParameters = new RMLPositionOutputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );
  } // end if


  
  // Phase sync makes the orientation correct to drive in a straight line
  reflexxesData_.flags.SynchronizationBehavior = 
    RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = MAX_SPEED * cos(PI/4.f);
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = (2.f*PI)/3.f;
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 1.;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = (2.f*PI)/3.f;
  

  // As the maximum jerk values are not known, this is just to try
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[1] = PI;

  // Set flag value to know if Reflexxes has been called yet
  reflexxesData_.outputParameters->NewPositionVector->VecData[0] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[1] = -99;


  // Result
  reflexxesData_.resultValue = 0;
} // End initReflexxes



/** Initialize class object with a request */
// TODO: change 3 booleans to 1 enum
void MobileBase::init(const ramp_msgs::TrajectoryRequest req) 
{
  //////////////////ROS_INFO("Entered MobileBase::init");
  //std::cout<<"\nRequest received: "<<utility_.toString(req)<<"\n";

  //if(req.bezierInfo.u_0 > 0)
    //std::cout<<"\nBezier Info passed in: "<<utility_.toString(req.bezierInfo);

  // Store the path
  path_ = req.path;

  // Set print
  print_ = req.print;
 
  // Set trajectory type
  //////////////ROS_INFO("req.type: %i", req.type);
  type_ = (TrajectoryType)req.type;
  //////////////ROS_INFO("type_: %i", type_);

  // Set segments
  segments_ = req_.segments;

  // Initialize Reflexxes
  initReflexxes();
 
  // Set the initial conditions of the reflexxes library
  setInitialMotion();

  // Set SelectionVector
  setSelectionVector();

  // Starting time
  t_started_ = ros::Time::now();

  // Set the time to cutoff generating points
  // Mostly for catching any small bugs that
  // make Reflexxes unable to find goal
  timeCutoff_ = ros::Duration(50);
  
  //////////////////ROS_INFO("Leaving MobileBase::init");
} // End init



/** This method sets the new target of Reflexxes */
// ********************** Add a check for if there is a target for a non-selected dimension that's
// different than its current value **********************
void MobileBase::setTarget(const ramp_msgs::MotionState& ms) 
{
  //////////////ROS_INFO("In MobileBase::setTarget");
  //////////////ROS_INFO("ms: %s", utility_.toString(ms).c_str());
  //////////////ROS_INFO("Prev: %s", utility_.toString(path_.points.at(i_kp_-1).motionState).c_str());
  
  double y_diff = path_.points.at(i_kp_).motionState.positions.at(1) - prevKP_.positions.at(1);
  double x_diff = path_.points.at(i_kp_).motionState.positions.at(0) - prevKP_.positions.at(0);
  bool x_diff_greater = fabs(x_diff) > fabs(y_diff);
 
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) 
  {
    //////////////ROS_INFO("i: %i", i);
    if(i == i_THETADOF_)
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = prevKP_.positions.at(2);
      reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(2);
    }    
    else if(x_diff_greater)
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = prevKP_.positions.at(0);
      reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(0);
    }
    else
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = prevKP_.positions.at(1);
      reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(1);
    }

    if(ms.velocities.size() > i) 
    {
      if(i==i_THETADOF_)
      {

        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = prevKP_.velocities.at(2);
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(2);
      }
      else if(x_diff_greater)
      {
        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = 
          prevKP_.velocities.at(0);
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(0);
      }
      else
      {
        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = 
          prevKP_.velocities.at(1);
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(1);
      }

      // If the target velocity == 0, Reflexxes throws an error so check for that
      if(fabs(ms.velocities.at(i)) < 0.000001)
      {
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = 0.000001;
      }

      ////////////////ROS_INFO("fabs( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] - reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]: %f", fabs( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] - reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]) );

      // Also check if there is some small floating point difference between target and max
      if( fabs( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] -
                reflexxesData_.inputParameters->MaxVelocityVector->VecData[i])    < 0.01)
      {
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = reflexxesData_.inputParameters->MaxVelocityVector->VecData[i];
      } // end if small difference
    } //end if target has velocities
  } // end for

  ////////////////ROS_INFO("Target V now: (%f, %f, %f", reflexxesData_.inputParameters->TargetVelocityVector->VecData[0], reflexxesData_.inputParameters->TargetVelocityVector->VecData[1], reflexxesData_.inputParameters->TargetVelocityVector->VecData[2]);

  // Phase sync makes the orientation correct to drive in a straight line
  reflexxesData_.flags.SynchronizationBehavior = 
    RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;

  ////printReflexxesSpinInfo();
  //////////////ROS_INFO("Target: %f", reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);

  //////////////ROS_INFO("Exiting MobileBase::setTarget");
} // End setTarget


void MobileBase::setMaxV(const double x_dot, const double theta_dot)
{
  //////////////ROS_INFO("In MobileBa::setMaxV");
  //////////////ROS_INFO("x_dot: %f theta_dot: %f", x_dot, theta_dot);


  /*
   * If max velocity == 0, Reflexxes throws an error
   * Check for that in all DOF
   */
  if(fabs(x_dot) < 0.000001)
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = 0.000001;
  }
  else
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = fabs(x_dot);
  }

  /*if(fabs(y_dot) < 0.000001)
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.000001;
  }
  else
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = y_dot;
  }*/

  if(fabs(theta_dot) < 0.000001)
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.000001;
  }
  else
  {
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = fabs(theta_dot);
  }
 


  // Check the current V to see if it is within a threshold of max V
  // If it is essentially the max, change the current to be the max to prevent Reflexxes error
  for(uint8_t i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++)
  {
    //////////////////ROS_INFO("i: %i", i);
    //////////////////ROS_INFO("reflexxesData_.inputParameters->CurrentVelocityVector->VecData[%i]: %f", i, reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
    //////////////////ROS_INFO("reflexxesData_.inputParameters->MaxVelocityVector->VecData[%i]: %f", i, reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]);
    //////////////////ROS_INFO("diff: %f", fabs(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] - reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]));


    if(fabs(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] - 
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]) < 0.01)
    {
      reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = 
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]-0.0001;
      //////////////////ROS_INFO("Current Now: %f", reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
    } // end if
  } // end for

  //////////////////ROS_INFO("Max V: (%f, %f, %f): ", reflexxesData_.inputParameters->MaxVelocityVector->VecData[0], reflexxesData_.inputParameters->MaxVelocityVector->VecData[1], reflexxesData_.inputParameters->MaxVelocityVector->VecData[2]);
 
  //////////////ROS_INFO("Exiting MobileBa::setMaxV");
} // End setMaxV







/** This method sets the SelectionVector for x,y trajectories */
void MobileBase::setSelectionVector() {

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = false;
} // End setSelectionVector




/** This method sets the SelectionVector for rotation parts */
void MobileBase::setSelectionVectorRotation() {
  reflexxesData_.inputParameters->SelectionVector->VecData[0] = false;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = true;
} // End setSelectionVectorRotation


/**
 * Initialize variables after receiving a service request
 * Set-up the input parameters
 **/
void MobileBase::setInitialMotion() 
{
  ////////////ROS_INFO("In MobileBase::setInitialMotion");
  
  // Initialise the time to use for each trajectory point
  timeFromStart_ = ros::Duration(0);
 
  i_kp_=1; 
  double y_diff = path_.points.at(i_kp_).motionState.positions.at(1) - path_.points.at(i_kp_-1).motionState.positions.at(1);
  double x_diff = path_.points.at(i_kp_).motionState.positions.at(0) - path_.points.at(i_kp_-1).motionState.positions.at(0);
  ////////////ROS_INFO("path_.points.at(i_kp_).motionState: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());
  ////////////ROS_INFO("path_.points.at(i_kp_-1).motionState: %s", utility_.toString(path_.points.at(i_kp_-1).motionState).c_str());
  double slope = y_diff / x_diff;

  double theta = utility_.findAngleFromAToB( path_.points.at(i_kp_-1).motionState.positions,
                                             path_.points.at(i_kp_).motionState.positions);

  ////////////ROS_INFO("x_diff: %f y_diff: %f", x_diff, y_diff);
  bool x_diff_greater = fabs(x_diff) > fabs(y_diff);
  
  // Set the positions of the robot as Reflexxes input
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) 
  {
    if(i == i_THETADOF_)
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(2);
    }
    else if(x_diff_greater)
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(0);
    }
    else
    {
      reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(1);
    }
  }

  // Set the current velocities of the robot as Reflexxes input
  if(path_.points.at(0).motionState.velocities.size() > 0) 
  {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) 
    {
      if(i == i_THETADOF_)
      {
        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points[0].motionState.velocities.at(2);
      }
      else if(x_diff_greater)
      {
        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points[0].motionState.velocities.at(0);
      }
      else
      {
        reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points[0].motionState.velocities.at(1);
      }
    }
  }
  else {//log some error
  }

  // Set the current accelerations of the robot as Reflexxes input
  if(path_.points.at(0).motionState.accelerations.size() > 0) 
  {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) 
    {
      if(i == i_THETADOF_)
      {
        reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points[0].motionState.accelerations.at(2);
      }
      else if(x_diff_greater)
      {
        reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points[0].motionState.accelerations.at(0);
      }
      else
      {
        reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points[0].motionState.accelerations.at(1);
      }
    }
  }
  else {//log some error
  }

  ////////////ROS_INFO("Exiting MobileBase::setInitialMotion");
} // End setCurrentMotion







/** Inserts a MotionState into the response trajectory */
void MobileBase::insertPoint(const ramp_msgs::MotionState& ms, ramp_msgs::TrajectoryResponse& res) 
{
  trajectory_msgs::JointTrajectoryPoint jp = utility_.getTrajectoryPoint(ms);
  jp.time_from_start = timeFromStart_;
  insertPoint(jp, res);
} // End insertPoint


/** Inserts a JointTrajectoryPoint at the back of response trajectory and sets Reflexxes to that point */
void MobileBase::insertPoint(const trajectory_msgs::JointTrajectoryPoint& jp, ramp_msgs::TrajectoryResponse& res) 
{
  res.trajectory.trajectory.points.push_back(jp);

  /** Update Reflexxes */
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);
  
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = jp.positions.at(0);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[1] = jp.positions.at(1);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = jp.positions.at(2);
  
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = jp.velocities.at(0);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = jp.velocities.at(1);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = jp.velocities.at(2);
  
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = jp.accelerations.at(0);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = jp.accelerations.at(1);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = jp.accelerations.at(2);


} // End insertPoint




/** Tests if a lambda value will have Bezier equations that are defined */
const bool MobileBase::lambdaOkay(const std::vector<ramp_msgs::MotionState> segment_points, const double lambda) const {
  ////////////ROS_INFO("In lambdaOkay, lambda: %f", lambda);
  ramp_msgs::MotionState X0, X1, X2, p0, p1, p2;

  p0 = segment_points.at(0);
  p1 = segment_points.at(1);
  p2 = segment_points.at(2);
  
  X1 = segment_points.at(1);


  // Find how far along segment we already are
  // Can use x or y...here we use x
  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  ////////////ROS_INFO("min_lambda in lambdaOkay: %f", min_lambda); 

  // TODO: Check for v
  if(lambda < min_lambda) 
  {
    return false;
  }
  

  
  double l_s1 = utility_.positionDistance(segment_points.at(1).positions, segment_points.at(0).positions);
  double l_s2 = utility_.positionDistance(segment_points.at(2).positions, segment_points.at(1).positions);

  if(l_s2 < l_s1) 
  {
    X2.positions.push_back( (1-lambda)*p1.positions.at(0) + lambda*p2.positions.at(0) );
    X2.positions.push_back( (1-lambda)*p1.positions.at(1) + lambda*p2.positions.at(1) );
    X2.positions.push_back(utility_.findAngleFromAToB(p1.positions, p2.positions));

    double theta = utility_.findAngleFromAToB(p0.positions, p1.positions);

    X0.positions.push_back( p1.positions.at(0) - l_s2*cos(theta) );
    X0.positions.push_back( p1.positions.at(1) - l_s2*sin(theta) );
    X0.positions.push_back( theta );
  }
  else 
  {
    X0.positions.push_back( (1-lambda)*p0.positions.at(0) + lambda*p1.positions.at(0) );
    X0.positions.push_back( (1-lambda)*p0.positions.at(1) + lambda*p1.positions.at(1) );
    X0.positions.push_back(utility_.findAngleFromAToB(p0.positions, p1.positions));

    double theta = utility_.findAngleFromAToB(p1.positions, p2.positions);

    X2.positions.push_back( p1.positions.at(0) + l_s2*cos(theta) );
    X2.positions.push_back( p1.positions.at(1) + l_s2*sin(theta) );
    X2.positions.push_back( theta );
  }


  // If both A and B will equal 0
  if(X1.positions.at(0) == ( (X0.positions.at(0) + X2.positions.at(0)) / 2. ) &&
      X1.positions.at(1) == ( (X0.positions.at(1) + X2.positions.at(1)) / 2. )) 
  {
    ////////////ROS_INFO("%f not okay", lambda);
    return false;
  }
  
  return true;
} // End lambdaOkay




/** Returns a lambda value that will lead to defined Bezier equations */
const double MobileBase::getControlPointLambda(const std::vector<ramp_msgs::MotionState> segment_points) const 
{
  ////////////ROS_INFO("In MobileBase::getControlPointLambda");

  std::vector<double> result;

  // Start transition trajectories right away,
  // otherwise, try to go straight for a while
  double lambda = type_ == TRANSITION ? 0.1 : 0.85;

  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  ////////////ROS_INFO("min_lambda: %f", min_lambda);

  if(min_lambda > 1) 
  {
    ////////ROS_ERROR("Minimum lambda: %f", min_lambda);
    ////////ROS_ERROR("Minimum lambda > 1");
    lambda = min_lambda;
  }
  else {

    bool loopedOnce=false;
    while(!lambdaOkay(segment_points, lambda) && !loopedOnce) 
    {
      if(type_ == TRANSITION) 
      {
        lambda+=0.05;
      }
      else 
      {
        lambda-=0.05;
      }

      if(lambda > 0.91) 
      {
        lambda = 0.1;
        loopedOnce = true;
      }
      else if(lambda < 0.05) 
      {
        lambda = 0.9;
        loopedOnce = true;
      }
    }
    ////////////ROS_INFO("lambda final: %f", lambda);
  }

  ////////////ROS_INFO("Exiting MobileBase::getControlPointLambda");
  return lambda;
} // End getControlPointLambda


const ramp_msgs::MotionState MobileBase::getMaxMS() const {
  //////////////////ROS_INFO("In getMaxMS()");
  ramp_msgs::MotionState result;

  
  if(req_.bezierCurves.size() > 0 && req_.bezierCurves.at(0).ms_maxVA.velocities.size() > 1)
  {
    result.velocities.push_back(req_.bezierCurves.at(0).ms_maxVA.velocities.at(0));
    result.velocities.push_back(req_.bezierCurves.at(0).ms_maxVA.velocities.at(0));
    result.velocities.push_back(req_.bezierCurves.at(0).ms_maxVA.velocities.at(1));
    result.accelerations.push_back(req_.bezierCurves.at(0).ms_maxVA.accelerations.at(0));
    result.accelerations.push_back(req_.bezierCurves.at(0).ms_maxVA.accelerations.at(0));
    result.accelerations.push_back(req_.bezierCurves.at(0).ms_maxVA.accelerations.at(1));
  }
  else
  {
    result.velocities.push_back(
        reflexxesData_.inputParameters->
        MaxVelocityVector->VecData[0]);
    result.velocities.push_back(
        reflexxesData_.inputParameters->
        MaxVelocityVector->VecData[0]);
    result.velocities.push_back(
        reflexxesData_.inputParameters->
        MaxVelocityVector->VecData[1]);
    result.accelerations.push_back(
        reflexxesData_.inputParameters->
        MaxAccelerationVector->VecData[0]);
    result.accelerations.push_back(
        reflexxesData_.inputParameters->
        MaxAccelerationVector->VecData[0]);
    result.accelerations.push_back(
        reflexxesData_.inputParameters->
        MaxAccelerationVector->VecData[1]);
    /*result.velocities.push_back(0.33);
    result.velocities.push_back(0.33);
    result.accelerations.push_back(1);
    result.accelerations.push_back(1);*/
  }

  //////////////////ROS_INFO("Leaving getMaxMS()");
  return result;
}


/** */
const std::vector<BezierCurve> MobileBase::bezier(ramp_msgs::Path& p, const bool only_curve) 
{
  //ROS_INFO("Entered MobileBase::bezier");

  std::vector<BezierCurve> result;

  ramp_msgs::Path p_copy = p;

  // Set the index of which knot point to stop at
  //int stop = (req_.type == TRANSITION) ? 3 : 2; 
  int stop = req_.bezierCurves.size()+1;
  //std::cout<<"\nstop: "<<stop;

  // TODO: Add a check to see if a curve exists in the request to prevent index out of bounds crashes

  // Find the first segment point for the curve
  // Increment index until the two points are different
  bool differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01;
  int inc = 2;
  while(!differentPoint && inc < p_copy.points.size()) {
    /*std::cout<<"\nPoints 0 and 1 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p)<<"\n";
    std::cout<<"\nSegment point 0: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(0));
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1))<<"\n";*/
    req_.bezierCurves.at(0).segmentPoints.at(1) = p_copy.points.at(inc).motionState;
    differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01;
    //std::cin.get();
    inc++;

    if(inc == p_copy.points.size()) 
    {
      //////////////ROS_INFO("Cannot plan Bezier, returning same path");
      type_ = HOLONOMIC;
      return result;
    }
  }

  // Find the second segment point for the curve
  // Increment index until the two points are different
  differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(2).positions) > 0.01;
  inc = 3;
  while(!differentPoint && inc < p_copy.points.size()) 
  {
    /*std::cout<<"\nPoints 1 and 2 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p);
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1));
    std::cout<<"\nSegment point 2: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(2))<<"\n";*/
    req_.bezierCurves.at(0).segmentPoints.at(2) = p_copy.points.at(inc).motionState;
    //std::cin.get();
    differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(2).positions) > 0.01;
    inc++;

    if(inc == p_copy.points.size()) 
    {
      //////////////ROS_INFO("Cannot plan Bezier, returning same path loop 2");
      type_ = HOLONOMIC;
      return result;
    }
  }

  ////////////ROS_INFO("stop: %i", stop);

  // Go through the path's knot points
  //std::cout<<"\np.points.size(): "<<p.points.size()<<"\n";
  for(uint8_t i=1;i<stop;i++) {
    //std::cout<<"\n---i: "<<(int)i<<"---\n";

    // Check that all of the points are different
    if(utility_.positionDistance(req_.bezierCurves.at(i-1).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(i-1).segmentPoints.at(1).positions) > 0.01 &&
        (utility_.positionDistance(req_.bezierCurves.at(i-1).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(i-1).segmentPoints.at(2).positions) > 0.01) )
    {
      //ROS_INFO("In if");

      BezierCurve bc;
      bc.print_ = print_;

      // Set segment points
      std::vector<ramp_msgs::MotionState> segment_points = 
        req_.bezierCurves.at(i-1).segmentPoints;
      
      double theta = utility_.findAngleFromAToB(
          segment_points.at(0).positions, segment_points.at(1).positions);
      double lambda;

      // If we are starting with a curve
      // For transition trajectories, the segment points are the 
      // control points, so we have all the info now
      if(req_.bezierCurves.at(0).u_0 > 0 && i==1) 
      {
        //std::cout<<"\nIn if transition or bezierStart\n";
        
        ramp_msgs::MotionState ms_maxVA = getMaxMS();
        
        lambda = (req_.bezierCurves.at(i-1).controlPoints.size() > 0) ?  
          req_.bezierCurves.at(i-1).l : getControlPointLambda(segment_points);


        // TODO: Make a method to return a BezierInitializer
        // TODO: Just use req_.bezierCurves?
        ramp_msgs::BezierCurve bi;
        bi.segmentPoints    = segment_points;
        bi.controlPoints    = req_.bezierCurves.at(i-1).controlPoints;
        bi.ms_maxVA         = ms_maxVA;
        bi.u_0              = req_.bezierCurves.at(i-1).u_0;
        bi.u_dot_0          = req_.bezierCurves.at(i-1).u_dot_0;
        bi.u_dot_max        = req_.bezierCurves.at(i-1).u_dot_max;
        bi.ms_begin         = p_copy.points.at(0).motionState;
        bi.l                = lambda;

        bc.init(bi, path_.points.at(0).motionState);
      } // end if bezierStart

      // If a "normal" bezier trajectory,
      else 
      {
        //ROS_INFO("In else a normal trajectory");

        // Get lambda value for segment points
        lambda = (req_.bezierCurves.at(i-1).controlPoints.size() > 0) ?  req_.bezierCurves.at(i-1).l :
                                                        getControlPointLambda(segment_points);
        ////////////ROS_INFO("lambda: %f", lambda);

        ramp_msgs::MotionState ms_maxVA = getMaxMS();

        // TODO: Make a method to return a BezierInitializer
        ramp_msgs::BezierCurve bi;
        bi.segmentPoints  = segment_points;
        bi.controlPoints  = req_.bezierCurves.at(i-1).controlPoints;
        bi.l              = lambda;
        bi.ms_maxVA       = ms_maxVA;

       

        bc.init(bi, path_.points.at(0).motionState);
      } // end else "normal" trajectory


      bool verified = bc.verify();
      // TODO: Implement break in case of infinite loop, print error
      while(lambdaOkay(bc.segmentPoints_, lambda) && lambda > 0.09 && lambda < 0.91 && !verified)
      {
        ////////////ROS_INFO("Lambda %f did not work", lambda);
        if(req_.type == TRANSITION)
        {
          lambda += 0.05;
        }
        else
        {
          lambda -= 0.05;
        }
        //////////////////ROS_INFO("New lambda: %f", lambda);
        ramp_msgs::BezierCurve bi;
        ramp_msgs::MotionState ms_maxVA = getMaxMS();
        bi.segmentPoints  = segment_points;
        bi.controlPoints  = req_.bezierCurves.at(i-1).controlPoints;
        bi.l              = lambda;
        bi.ms_maxVA       = ms_maxVA;

        bc.segmentPoints_.clear();
        bc.controlPoints_.clear();
        bc.init(bi, path_.points.at(0).motionState);
       
        
        verified = bc.verify();
      }

      // Verify the curve
      if(verified) 
      {
        //ROS_INFO("Curve is verified, generating points");

        // Generate the curve
        bc.generateCurveOOP();
        result.push_back(bc);
      }
      
      else if(type_ == TRANSITION) 
      {
        //ROS_INFO("Curve not verified, doing a transition so setting 0 velocity for KP: %s", utility_.toString(path_.points.at(1).motionState).c_str());

        uint8_t num_dof = path_.points.at(1).motionState.velocities.size();
        for(uint8_t i_v=0;i_v<num_dof;i_v++) 
        {
          path_.points.at(1).motionState.velocities.at(i_v) = 0;
        }

        type_ = HOLONOMIC;
      } // end else if transition
      else 
      {
        //ROS_INFO("Curve not verified, but not a transition trajectory");
        type_ = HOLONOMIC;
      }
    } // end if
    else 
    {
      //ROS_WARN("Two of the three segment points for Bezier curve are too close");
      type_ = HOLONOMIC;
    }
  } // end for

  //ROS_INFO("Outside of for");

  // Set Path p's knot point indices
  if(type_ != HOLONOMIC) 
  {
    if(type_ == TRANSITION) 
    {
      //ROS_INFO("In type == transition");
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.erase(p.points.begin()+2);
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }

    // If we have more than 1 curve
    else if(req_.bezierCurves.size() > 1) 
    {
      //////////////////ROS_INFO("In else if bezierInfo.size()>1");

      //////////////////ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+2) ).c_str());
      p.points.erase( p.points.begin()+2 );
      //////////////////ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin()+1 );
      // Insert the 1st curve's last CP
      p.points.insert( p.points.begin()+1, 
          utility_.getKnotPoint( result.at(0).points_.at(result.at(0).points_.size()-1)));

      //////////////////ROS_INFO("Path p: %s", utility_.toString(p).c_str());
      
      // Insert the 2nd curve's 1st and last CPs
      p.points.insert(p.points.begin()+2, utility_.getKnotPoint(result.at(1).points_.at(0)));
      p.points.insert(p.points.begin()+3, 
          utility_.getKnotPoint(result.at(1).points_.at(result.at(1).points_.size()-1)));

      if(!planning_full_)
      {
        segments_++;
      }
    }

    // If already moving on curve
    else if(req_.bezierCurves.at(0).u_0 > 0) 
    {
      //////////////////ROS_INFO("In else if bezierStart");
      p.points.erase( p.points.begin() + 1 );
      p.points.insert(p.points.begin()+1, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }

    else if(utility_.positionDistance( p.points.at(1).motionState.positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01)
    {
      //////////////////ROS_INFO("In else if Knot Point 1 != segment point 1");
      //////////////////ROS_INFO("Knot Point 1: %s\nSegment Point 1: %s", utility_.toString(p.points.at(1).motionState).c_str(), utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1)).c_str());
      // Don't erase anything
      // Insert
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
      
      if(!planning_full_)
      {
        segments_++;
        segments_++;
      }
    }

    // Else not on a curve, but a curve exists on the trajectory
    // Remove the 2nd knot point and replace it with start and end of the upcoming curve
    else 
    {
      //////////////////ROS_INFO("In else");
      //////////////////ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin() + 1 );


      // Insert
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));

      if(!planning_full_)
      {
        segments_++;
        segments_++;
      }
    }
  } // end if not all straight segments

  ////////////////ROS_INFO("Exiting MobileBase::bezier");
  return result;
} // End bezier





void MobileBase::bezierOOP(ramp_msgs::Path& p, bool only_curve, std::vector<BezierCurve>& result)
{
  ////////////ROS_INFO("Entered MobileBase::bezier");

  ramp_msgs::Path p_copy = p;

  // Set the index of which knot point to stop at
  //int stop = (req_.type == TRANSITION) ? 3 : 2; 
  int stop = req_.bezierCurves.size()+1;
  //std::cout<<"\nstop: "<<stop;

  // TODO: Add a check to see if a curve exists in the request to prevent index out of bounds crashes

  // Find the first segment point for the curve
  // Increment index until the two points are different
  bool differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01;
  int inc = 2;
  while(!differentPoint && inc < p_copy.points.size()) {
    /*std::cout<<"\nPoints 0 and 1 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p)<<"\n";
    std::cout<<"\nSegment point 0: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(0));
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1))<<"\n";*/
    req_.bezierCurves.at(0).segmentPoints.at(1) = p_copy.points.at(inc).motionState;
    differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01;
    //std::cin.get();
    inc++;

    if(inc == p_copy.points.size()) 
    {
      //////////////ROS_INFO("Cannot plan Bezier, returning same path");
      type_ = HOLONOMIC;
      return;
    }
  }

  // Find the second segment point for the curve
  // Increment index until the two points are different
  differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(2).positions) > 0.01;
  inc = 3;
  while(!differentPoint && inc < p_copy.points.size()) 
  {
    /*std::cout<<"\nPoints 1 and 2 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p);
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1));
    std::cout<<"\nSegment point 2: "<<utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(2))<<"\n";*/
    req_.bezierCurves.at(0).segmentPoints.at(2) = p_copy.points.at(inc).motionState;
    //std::cin.get();
    differentPoint = utility_.positionDistance(req_.bezierCurves.at(0).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(0).segmentPoints.at(2).positions) > 0.01;
    inc++;

    if(inc == p_copy.points.size()) 
    {
      //////////////ROS_INFO("Cannot plan Bezier, returning same path loop 2");
      type_ = HOLONOMIC;
      return;
    }
  }

  ////////////ROS_INFO("stop: %i", stop);

  // Go through the path's knot points
  //std::cout<<"\np.points.size(): "<<p.points.size()<<"\n";
  for(uint8_t i=1;i<stop;i++) {
    //std::cout<<"\n---i: "<<(int)i<<"---\n";

    // Check that all of the points are different
    if(utility_.positionDistance(req_.bezierCurves.at(i-1).segmentPoints.at(0).positions, 
          req_.bezierCurves.at(i-1).segmentPoints.at(1).positions) > 0.01 &&
        (utility_.positionDistance(req_.bezierCurves.at(i-1).segmentPoints.at(1).positions, 
          req_.bezierCurves.at(i-1).segmentPoints.at(2).positions) > 0.01) )
    {
      ////////////ROS_INFO("In if");

      BezierCurve bc;
      bc.print_ = print_;

      // Set segment points
      std::vector<ramp_msgs::MotionState> segment_points = 
        req_.bezierCurves.at(i-1).segmentPoints;
      
      double theta = utility_.findAngleFromAToB(
          segment_points.at(0).positions, segment_points.at(1).positions);
      double lambda;

      // If we are starting with a curve
      // For transition trajectories, the segment points are the 
      // control points, so we have all the info now
      if(req_.bezierCurves.at(0).u_0 > 0 && i==1) 
      {
        //std::cout<<"\nIn if transition or bezierStart\n";
        
        ramp_msgs::MotionState ms_maxVA = getMaxMS();
        
        lambda = (req_.bezierCurves.at(i-1).controlPoints.size() > 0) ?  
          req_.bezierCurves.at(i-1).l : getControlPointLambda(segment_points);


        // TODO: Make a method to return a BezierInitializer
        // TODO: Just use req_.bezierCurves?
        ramp_msgs::BezierCurve bi;
        bi.segmentPoints    = segment_points;
        bi.controlPoints    = req_.bezierCurves.at(i-1).controlPoints;
        bi.ms_maxVA         = ms_maxVA;
        bi.u_0              = req_.bezierCurves.at(i-1).u_0;
        bi.u_dot_0          = req_.bezierCurves.at(i-1).u_dot_0;
        bi.u_dot_max        = req_.bezierCurves.at(i-1).u_dot_max;
        bi.ms_begin         = p_copy.points.at(0).motionState;
        bi.l                = lambda;

        bc.init(bi, path_.points.at(0).motionState);
      } // end if bezierStart

      // If a "normal" bezier trajectory,
      else 
      {
        ////////////ROS_INFO("In else a normal trajectory");

        // Get lambda value for segment points
        lambda = (req_.bezierCurves.at(i-1).controlPoints.size() > 0) ?  req_.bezierCurves.at(i-1).l :
                                                        getControlPointLambda(segment_points);
        ////////////ROS_INFO("lambda: %f", lambda);

        ramp_msgs::MotionState ms_maxVA = getMaxMS();

        // TODO: Make a method to return a BezierInitializer
        ramp_msgs::BezierCurve bi;
        bi.segmentPoints  = segment_points;
        bi.controlPoints  = req_.bezierCurves.at(i-1).controlPoints;
        bi.l              = lambda;
        bi.ms_maxVA       = ms_maxVA;

       

        bc.init(bi, path_.points.at(0).motionState);
      } // end else "normal" trajectory


      bool verified = bc.verify();
      // TODO: Implement break in case of infinite loop, print error
      while(lambdaOkay(bc.segmentPoints_, lambda) && lambda > 0.09 && lambda < 0.91 && !verified)
      {
        ////////////ROS_INFO("Lambda %f did not work", lambda);
        if(req_.type == TRANSITION)
        {
          lambda += 0.05;
        }
        else
        {
          lambda -= 0.05;
        }
        //////////////////ROS_INFO("New lambda: %f", lambda);
        ramp_msgs::BezierCurve bi;
        ramp_msgs::MotionState ms_maxVA = getMaxMS();
        bi.segmentPoints  = segment_points;
        bi.controlPoints  = req_.bezierCurves.at(i-1).controlPoints;
        bi.l              = lambda;
        bi.ms_maxVA       = ms_maxVA;

        bc.segmentPoints_.clear();
        bc.controlPoints_.clear();
        bc.init(bi, path_.points.at(0).motionState);
       
        
        verified = bc.verify();
      }

      // Verify the curve
      if(verified) 
      {
        ////////////ROS_INFO("Curve is verified, generating points");

        // Generate the curve
        bc.generateCurveOOP();
        result.push_back(bc);
      }
      
      else if(type_ == TRANSITION) 
      {
        ////////////ROS_INFO("Curve not verified, doing a transition so setting 0 velocity for KP: %s", utility_.toString(path_.points.at(1).motionState).c_str());

        uint8_t num_dof = path_.points.at(1).motionState.velocities.size();
        for(uint8_t i_v=0;i_v<num_dof;i_v++) 
        {
          path_.points.at(1).motionState.velocities.at(i_v) = 0;
        }

        type_ = HOLONOMIC;
      } // end else if transition
      else 
      {
        ////////////ROS_INFO("Curve not verified, but not a transition trajectory");
        type_ = HOLONOMIC;
      }
    } // end if
    else 
    {
      ////////ROS_WARN("Two of the three segment points for Bezier curve are too close");
      type_ = HOLONOMIC;
    }
  } // end for

  ////////////ROS_INFO("Outside of for");

  // Set Path p's knot point indices
  if(type_ != HOLONOMIC) 
  {
    if(type_ == TRANSITION) 
    {
      //////////////////ROS_INFO("In type == transition");
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.erase(p.points.begin()+2);
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }

    // If we have more than 1 curve
    else if(req_.bezierCurves.size() > 1) 
    {
      //////////////////ROS_INFO("In else if bezierInfo.size()>1");

      //////////////////ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+2) ).c_str());
      p.points.erase( p.points.begin()+2 );
      //////////////////ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin()+1 );
      // Insert the 1st curve's last CP
      p.points.insert( p.points.begin()+1, 
          utility_.getKnotPoint( result.at(0).points_.at(result.at(0).points_.size()-1)));

      //////////////////ROS_INFO("Path p: %s", utility_.toString(p).c_str());
      
      // Insert the 2nd curve's 1st and last CPs
      p.points.insert(p.points.begin()+2, utility_.getKnotPoint(result.at(1).points_.at(0)));
      p.points.insert(p.points.begin()+3, 
          utility_.getKnotPoint(result.at(1).points_.at(result.at(1).points_.size()-1)));

      if(!planning_full_)
      {
        segments_++;
      }
    }

    // If already moving on curve
    else if(req_.bezierCurves.at(0).u_0 > 0) 
    {
      //////////////////ROS_INFO("In else if bezierStart");
      p.points.erase( p.points.begin() + 1 );
      p.points.insert(p.points.begin()+1, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }

    else if(utility_.positionDistance( p.points.at(1).motionState.positions, 
          req_.bezierCurves.at(0).segmentPoints.at(1).positions) > 0.01)
    {
      //////////////////ROS_INFO("In else if Knot Point 1 != segment point 1");
      //////////////////ROS_INFO("Knot Point 1: %s\nSegment Point 1: %s", utility_.toString(p.points.at(1).motionState).c_str(), utility_.toString(req_.bezierCurves.at(0).segmentPoints.at(1)).c_str());
      // Don't erase anything
      // Insert
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
      
      if(!planning_full_)
      {
        segments_++;
        segments_++;
      }
    }

    // Else not on a curve, but a curve exists on the trajectory
    // Remove the 2nd knot point and replace it with start and end of the upcoming curve
    else 
    {
      //////////////////ROS_INFO("In else");
      //////////////////ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin() + 1 );


      // Insert
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));

      if(!planning_full_)
      {
        segments_++;
        segments_++;
      }
    }
  } // end if not all straight segments

  ////////////////ROS_INFO("Exiting MobileBase::bezier");
}




/** Print Reflexxes input and output for the latest call */
void MobileBase::printReflexxesSpinInfo() const 
{
  std::cout<<"\n\n*****************************************************************************";
  std::cout<<"\nCalled reflexxes with input:";
  
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]: "<<
                reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentPositionVector->VecData[1];
  //std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[2]: "<<
  //              reflexxesData_.inputParameters->CurrentPositionVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1];
  //std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[2]: "<<
  //              reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1];
  //std::cout<<"\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2]: "<<
  //              reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2];
  
  
  std::cout<<"\n\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[1]: "<<
                reflexxesData_.inputParameters->MaxVelocityVector->VecData[1];
  //std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[2]: "<<
  //              reflexxesData_.inputParameters->MaxVelocityVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[1]: "<<
                reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1];
  //std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[2]: "<<
  //              reflexxesData_.inputParameters->MaxAccelerationVector->VecData[2];
  
  
  std::cout<<"\n\nOutput: ";
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewPositionVector->VecData[1];
  //std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[2]: "<<
  //              reflexxesData_.outputParameters->NewPositionVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.outputParameters->NewVelocityVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewVelocityVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewVelocityVector->VecData[1];
  //std::cout<<"\nreflexxesData_.outputParameters->NewVelocityVector->VecData[2]: "<<
  //              reflexxesData_.outputParameters->NewVelocityVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewAccelerationVector->VecData[1];
  //std::cout<<"\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[2]: "<<
  //              reflexxesData_.outputParameters->NewAccelerationVector->VecData[2];

  std::cout<<"\n\nResult value: "<<reflexxesData_.resultValue;
  std::cout<<"\nFinalStateReached: "<<finalStateReached();
  std::cout<<"\n*****************************************************************************";
} // End printReflexxesInfo




/** Execute one iteration of the Reflexxes control function */
const trajectory_msgs::JointTrajectoryPoint MobileBase::spinOnce(bool vertical_line) 
{

  // Calling the Reflexxes OTG algorithm
  reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, 
                                                  reflexxesData_.outputParameters, 
                                                  reflexxesData_.flags);


  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(reflexxesData_, vertical_line);

  //printReflexxesSpinInfo();


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = 
    *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
    *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
    *reflexxesData_.outputParameters->NewAccelerationVector;

  return point;
} // End spinOnce




/** Given Reflexxes data, return a trajectory point */
const trajectory_msgs::JointTrajectoryPoint MobileBase::buildTrajectoryPoint(const ReflexxesData data, bool vertical_line) 
{
  ////////////ROS_INFO("In MobileBase::buildTrajectoryPoint");
  //printReflexxesSpinInfo();

  trajectory_msgs::JointTrajectoryPoint point;

  ////////////ROS_INFO("path_.points.at(i_kp_).motionState: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());
  ////////////ROS_INFO("path_.points.at(i_kp_-1).motionState: %s", utility_.toString(path_.points.at(i_kp_-1).motionState).c_str());
  
  double y_diff = path_.points.at(i_kp_).motionState.positions.at(1) - path_.points.at(i_kp_-1).motionState.positions.at(1);
  double x_diff = path_.points.at(i_kp_).motionState.positions.at(0) - path_.points.at(i_kp_-1).motionState.positions.at(0);
  double slope = y_diff / x_diff;

  double theta = utility_.findAngleFromAToB( path_.points.at(i_kp_-1).motionState.positions,
                                             path_.points.at(i_kp_).motionState.positions);

  ////////////ROS_INFO("x_diff: %f y_diff: %f", x_diff, y_diff);
  bool x_diff_greater = fabs(x_diff) > fabs(y_diff);

  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) 
  {
    ////////////ROS_INFO("i: %i", i);

    ////////////ROS_INFO("slope: %f theta: %f path_points.at(i_kp-1): %f", slope, theta, path_.points.at(i_kp_-1).motionState.positions.at(1));

    // If Reflexxes has not been called yet
    if(data.outputParameters->NewPositionVector->VecData[0] == -99) 
    {
      ////////////ROS_INFO("In if");
      if(i==i_THETADOF_)
      {
        point.positions.push_back(data.inputParameters->CurrentPositionVector->VecData[i]);
        point.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[i]);
        point.accelerations.push_back(data.inputParameters->CurrentAccelerationVector->VecData[i]);
      }
      
      else
      {
        if(x_diff_greater)
        {
          ////////////ROS_INFO("In x_diff_greater");
          point.positions.push_back(data.inputParameters->CurrentPositionVector->VecData[i]);
          point.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[i]);
          point.accelerations.push_back(data.inputParameters->CurrentAccelerationVector->VecData[i]);
          
          
          // Compute y from x
          double b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                      (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
          double y = slope*point.positions.at(0) + b;
          double y_dot = point.velocities.at(0) * tan(theta);
          double y_ddot = 0;
          
          
          point.positions.push_back(y);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(y_ddot);
        }
        else
        {
          ////////////ROS_INFO("In y_diff_greater");
          double y = data.inputParameters->CurrentPositionVector->VecData[i];
          double y_dot = data.inputParameters->CurrentVelocityVector->VecData[i];
          double y_ddot = data.inputParameters->CurrentAccelerationVector->VecData[i];
          double b, x, x_dot, x_ddot;

          // Compute x from y
          if(std::isinf(slope))
          {
            ////////////ROS_INFO("In if isinf(slope), point: %s", utility_.toString(path_.points.at(0).motionState).c_str());
            x = path_.points.at(0).motionState.positions.at(0);
            x_dot = 0.f;
            x_ddot = 0.f;
          }
          else
          {
            ////////////ROS_INFO("In else");
            b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                        (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
            x = (y - b) / slope;
            x_dot = y_dot / tan(theta);
            x_ddot = 0;
          }

          ////////////ROS_INFO("b: %f", b);
          ////////////ROS_INFO("x: %f x_dot: %f x_ddot: %f", x, x_dot, x_ddot);
          ////////////ROS_INFO("y: %f y_dot: %f y_ddot: %f", y, y_dot, y_ddot);
          
          point.positions.push_back(x);
          point.positions.push_back(y);
          point.velocities.push_back(x_dot);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(x_ddot);
          point.accelerations.push_back(y_ddot);
        } // end else if y_diff>x_diff
      }
      // Compute y from x
      /*if(i == i_XDOF_)
      {
        //////////////ROS_INFO("slope: %f theta: %f path_points.at(i_kp-1): %f", slope, theta, 
            path_.points.at(i_kp_-1).motionState.positions.at(1));
        double y = slope*point.positions.at(0) + path_.points.at(0).motionState.positions.at(1);
        double y_dot = point.velocities.at(0) * tan(theta);
        double y_ddot = 0;

        point.positions.push_back(y);
        point.velocities.push_back(y_dot);
        point.accelerations.push_back(y_ddot);
      }*/
    } // end if reflexxes not called yet

    // If selection vector is true
    else if(reflexxesData_.inputParameters->SelectionVector->VecData[i]) 
    {
      ////////////////ROS_INFO("In else if");
      if(i==i_THETADOF_)
      {
        ////////////////ROS_INFO("In i_THETADOF_");
        point.positions.push_back(data.outputParameters->NewPositionVector->VecData[i]);
        point.velocities.push_back(data.outputParameters->NewVelocityVector->VecData[i]);
        point.accelerations.push_back(data.outputParameters->NewAccelerationVector->VecData[i]);
      }
  
      else
      {
        if(x_diff_greater)
        {
          ////////////////ROS_INFO("In x_diff_greater");
          point.positions.push_back(data.outputParameters->NewPositionVector->VecData[i]);
          point.velocities.push_back(data.outputParameters->NewVelocityVector->VecData[i]);
          point.accelerations.push_back(data.outputParameters->NewAccelerationVector->VecData[i]);
          
          
          // Compute y from x
          double b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                      (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
          double y = slope*point.positions.at(0) + b;
          double y_dot = point.velocities.at(0) * tan(theta);
          double y_ddot = 0;
          
          
          point.positions.push_back(y);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(y_ddot);
        }
        else
        {
          ////////////////ROS_INFO("In y_diff_greater");
          double y = data.outputParameters->NewPositionVector->VecData[i];
          double y_dot = data.outputParameters->NewVelocityVector->VecData[i];
          double y_ddot = data.outputParameters->NewAccelerationVector->VecData[i];
          double b, x, x_dot, x_ddot;

          // Compute x from y
          if(std::isinf(slope))
          {
            x = prevKP_.positions.at(0);
            x_dot = 0.f;
            x_ddot = 0.f;
          }
          else
          {
            b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                        (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
            x = (y - b) / slope;
            x_dot = y_dot / tan(theta);
            x_ddot = 0;
          }

          ////////////////ROS_INFO("b: %f y: %f y_dot: %f y_ddot: %f", b, y, y_dot, y_ddot);
          
          point.positions.push_back(x);
          point.positions.push_back(y);
          point.velocities.push_back(x_dot);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(x_ddot);
          point.accelerations.push_back(y_ddot);
        } // end else if y_diff>x_diff
      } // end else if selection vector true
    } // end if selection vector is true

    // Else if at theta
    else if(i == i_THETADOF_)
    { 
      ////////////////ROS_INFO("In else if i==1");
      double theta = utility_.findAngleFromAToB( path_.points.at(i_kp_-1).motionState.positions,
                                                 path_.points.at(i_kp_).motionState.positions);
                                                 //data.outputParameters->NewPositionVector->VecData[0],
                                                 //data.outputParameters->NewPositionVector->VecData[1]);




      ////////////////ROS_INFO("theta: %f", theta);
      ////////////////ROS_INFO("data.inputParameters->CurrentPositionVector->VecData[1]: %f", data.inputParameters->CurrentPositionVector->VecData[1]);
      // Get angular velocity
      double w = utility_.findDistanceBetweenAngles(data.inputParameters->CurrentPositionVector->VecData[1], theta) /
        CYCLE_TIME_IN_SECONDS;


      // Push on p,v,a
      // TODO: Manually keep acceleration?
      point.positions.push_back(theta);
      point.velocities.push_back(w);
      point.accelerations.push_back(0);

      // Set values in Reflexxes
      // TODO: Necessary?
      reflexxesData_.outputParameters->NewPositionVector->VecData[i] = theta;
      reflexxesData_.outputParameters->NewVelocityVector->VecData[i] = w;
    } // end if at theta

    // Else, just push on the current value
    else 
    {
      ////////////////ROS_INFO("In else");
      if(i==i_THETADOF_)
      {
        ////////////////ROS_INFO("In i_THETADOF_");
        point.positions.push_back(
            reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
        point.velocities.push_back(
            reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
        point.accelerations.push_back(
            reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i]);
      }
      
      else
      {
        ////////////////ROS_INFO("In i_XDOF_");
        if(x_diff_greater)
        {
          ////////////////ROS_INFO("In x_diff_greater");
          point.positions.push_back(data.outputParameters->NewPositionVector->VecData[i]);
          point.velocities.push_back(data.outputParameters->NewVelocityVector->VecData[i]);
          point.accelerations.push_back(data.outputParameters->NewAccelerationVector->VecData[i]);
          
          
          // Compute y from x
          double b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                      (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
          double y = slope*point.positions.at(0) + b;
          double y_dot = point.velocities.at(0) * tan(theta);
          double y_ddot = 0;
          
          
          point.positions.push_back(y);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(y_ddot);
        }
        else
        {
          ////////////////ROS_INFO("In y_diff_greater");
          double y = data.outputParameters->NewPositionVector->VecData[i];
          double y_dot = data.outputParameters->NewVelocityVector->VecData[i];
          double y_ddot = data.outputParameters->NewAccelerationVector->VecData[i];
          ////////////////ROS_INFO("y: %f y_dot: %f y_ddot: %f", y, y_dot, y_ddot);
          
          double b, x, x_dot, x_ddot;

          // Compute x from y
          if(std::isinf(slope))
          {
            x = prevKP_.positions.at(0);
            x_dot = 0.f;
            x_ddot = 0.f;
          }
          else
          {
            b = path_.points.at(i_kp_-1).motionState.positions.at(1) - 
                        (path_.points.at(i_kp_-1).motionState.positions.at(0)*slope);
            x = (y - b) / slope;
            x_dot = y_dot / tan(theta);
            x_ddot = 0;
          }
          
          ////////////////ROS_INFO("slope: %f theta: %f path_points.at(i_kp-1): %f", slope, theta, path_.points.at(i_kp_-1).motionState.positions.at(1));

          point.positions.push_back(x);
          point.positions.push_back(y);
          point.velocities.push_back(x_dot);
          point.velocities.push_back(y_dot);
          point.accelerations.push_back(x_ddot);
          point.accelerations.push_back(y_ddot);
        } // end else y_diff>x_diff
      } // end else not at theta
    } // end else selection vector false
  } // end for 

  ////////////ROS_INFO("Point built: %s", utility_.toString(point).c_str());

  // The timeFromStart_ is the time of the previous point 
  // plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  return point;
} // End buildTrajectoryPoint






const std::vector<uint8_t> MobileBase::getCurveKPs(const std::vector<BezierCurve> curves) const {
  std::vector<uint8_t> result;
  
  for(uint8_t i_c=0; i_c < curves.size(); i_c++) {
    //////////////////ROS_INFO("curves.at(%i): %s", (int)i_c, utility_.toString(curves.at(i_c).points_.at(0)).c_str());
    
    for(uint8_t i_kp=0;i_kp < path_.points.size();i_kp++) {
      //////////////////ROS_INFO("path.poinst.at(%i): %s", (int)i_kp, utility_.toString(path_.points.at(i_kp).motionState).c_str());
      
      if(utility_.positionDistance(curves.at(i_c).points_.at(0).positions,
                                   path_.points.at(i_kp).motionState.positions) < 0.0001)
      {
        result.push_back(i_kp);
        i_kp = path_.points.size();
      } // end if
      else {
      }
    } // end for kp
  } // end for i_c

  return result;
} // End getCurveKPs




bool MobileBase::checkSpeed(const ramp_msgs::Path p, const std::vector<uint8_t> i_cs)
{
  //////////////////ROS_INFO("In MobileBase::checkSpeed");
  for(uint8_t i=0;i<p.points.size()-1;i++)
  {
    //////////////////ROS_INFO("i: %i", i);
    if(i_cs.size() == 0 || (i_cs.size() > 0 && i != i_cs.at(0)))
    {
      if( p.points.at(i).motionState.velocities.size() > 0 &&
          fabs(p.points.at(i).motionState.velocities.at(0)) > 0.0001 &&
          fabs(p.points.at(i).motionState.velocities.at(1)) > 0.0001)
      {
        double delta_x = fabs(p.points.at(i+1).motionState.positions.at(0) - 
          p.points.at(i).motionState.positions.at(0));
        double delta_y = fabs(p.points.at(i+1).motionState.positions.at(1) - 
          p.points.at(i).motionState.positions.at(1));

        double max_gain_x = fabs(p.points.at(i).motionState.velocities.at(0)) * CYCLE_TIME_IN_SECONDS;
        double max_gain_y = fabs(p.points.at(i).motionState.velocities.at(1)) * CYCLE_TIME_IN_SECONDS;

        if( (max_gain_x > delta_x) ||
            (max_gain_y > delta_y) )
        {
          ////////ROS_WARN("The speed of knot point %i is high enough to overshoot next knot point.\nCheck this if there are issues with trajectory request", i);
          return false;
        }
      } // end if velocity size > 0
    }
  } // end for

  //////////////////ROS_INFO("Exiting MobileBase::checkSpeed");
  return true;
}



// Service callback, the input is a path and the output a trajectory
bool MobileBase::trajectoryRequest(ramp_msgs::TrajectoryRequest& req, ramp_msgs::TrajectoryResponse& res) 
{
  //////////ROS_INFO("In MobileBase::trajectoryRequest");
  ////////////ROS_INFO("type_: %i HOLONOMIC: %i", req.type, HOLONOMIC); 

  // If there's less than 3 points, make it have straight segments
  // if req_.segments == 1
  if( req.path.points.size() < 3 || req.segments == 1 ) 
  {
    //////ROS_WARN("Changing type to HOLONOMIC");
    req.type = HOLONOMIC;
    req.segments++;
  }

  // If there's 2 points and they have the same x,y position
  if(req.path.points.size() == 2 && 
      utility_.positionDistance(req.path.points.at(0).motionState.positions,
                                req.path.points.at(1).motionState.positions) < 0.001)
  {
    res.trajectory.trajectory.points.push_back(utility_.getTrajectoryPoint(req.path.points.at(0).motionState));
    res.trajectory.i_knotPoints.push_back(0);
    return true;
  }

  if(utility_.getEuclideanDist( req.path.points.at(0).motionState.positions, 
        req.path.points.at(1).motionState.positions) < 0.001)
  {
    req.path.points.erase(req.path.points.begin());
  }
  

  // Initialize with request
  init(req);
  req_ = req;
  
  // If using holonomic motion, set the knot velocities to all equal 0
  if(type_ == HOLONOMIC)
  {
    for(int i=1;i<path_.points.size();i++)
    {
      //////////////ROS_INFO("i: %i path_.points.size(): %i", i, (int)path_.points.size());
      double theta, theta_next, delta_theta = 0.f;
      if(i < path_.points.size()-1)
      {
        theta = fabs(utility_.findAngleFromAToB(path_.points.at(i-1).motionState.positions, 
            path_.points.at(i).motionState.positions));
        theta_next = fabs(utility_.findAngleFromAToB(path_.points.at(i).motionState.positions, 
            path_.points.at(i+1).motionState.positions));
        delta_theta = fabs(utility_.findDistanceBetweenAngles(theta, theta_next));
        //////////////ROS_INFO("theta: %f delta_theta: %f", theta, delta_theta);
      }

      if(delta_theta > 0.2)
      {
        //////////////ROS_INFO("Setting v=0 for this KP");
        for(int j=0;j<path_.points.at(i).motionState.positions.size();j++)
        {
          if(j < path_.points.at(i).motionState.velocities.size())
          {
            path_.points.at(i).motionState.velocities.at(j) = 0;
          }
          else
          {
            path_.points.at(i).motionState.velocities.push_back(0);
          }
        }
      }
    }
  }



  planning_full_ = segments_ == 0;
  /*if(planning_full_)
  {
    //////////////////ROS_INFO("req.segments == 0 - Getting trajectory for whole path");
  }
  else
  {
    //////////////////ROS_INFO("Not planning full, segments_: %i req_.segments: %i", (int)segments_, (int)req_.segments);
  }*/

  // Set start time
  t_started_ = ros::Time::now();

  std::vector<BezierCurve> curves;

  // Use Bezier curves to smooth path
  if(type_ != HOLONOMIC) 
  {
    //ROS_INFO("Path before Bezier: %s", utility_.toString(path_).c_str());
    //curves = bezier(path_, type_ == TRANSITION);
    bezierOOP(path_, type_ == TRANSITION, curves);
    //ROS_INFO("Path after Bezier: %s", utility_.toString(path_).c_str());
    setInitialMotion();


    // Currently adding 0 for both because 
    i_cs = getCurveKPs(curves);
    ////////////ROS_INFO("Curve indices: ");
    for(int i=0;i<i_cs.size();i++) {
      ////////////ROS_INFO("i_cs[%i]: %i", i, i_cs.at(i));
    }
  } // end if curves

  if(curves.size() == 0) 
  {
    type_ = HOLONOMIC;
  }

  // Print curves
  /*if(print_) {
    for(int c=0;c<curves.size();c++) {
      std::cout<<"\nCurve "<<c<<": ";
      for(int p=0;p<curves.at(c).points_.size();p++) {
        std::cout<<"\n"<<utility_.toString(curves.at(c).points_.at(p));
      }
    }
  }*/
 

  // Push 0 onto knot point indices
  res.trajectory.i_knotPoints.push_back(0);

  /*if(!checkSpeed(path_, i_cs))
  {
    //////////////ROS_INFO("Check speed is false! Removing knot point 1");
    path_.points.erase(path_.points.begin()+1);
    if(i_cs.size() > 0)
    {
      i_cs.at(0)--;
    }
  }*/


  if(planning_full_)
  {
    segments_ = path_.points.size();
  }


 
  ////////////////ROS_INFO("About to start generating points, segments_: %i", segments_);
  uint8_t c=0;
  // Go through every knotpoint in the path
  // (or until timeCutoff has been reached)
  //for (i_kp_ = 1; i_kp_<path_.points.size(); i_kp_++) 
  for (i_kp_ = 1; i_kp_<segments_; i_kp_++) 
  {
    ////////ROS_INFO("i_kp_: %i", (int)i_kp_);
    reflexxesData_.resultValue = 0;

    // Push the initial state onto trajectory
    // And set previous knot point
    if(i_kp_ == 1) 
    {
      res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(reflexxesData_));
      prevKP_ = res.trajectory.trajectory.points.at(0);
    }
    
    double theta = utility_.findAngleFromAToB(prevKP_.positions, path_.points.at(i_kp_).motionState.positions);
    ////////ROS_INFO("path_.points.at(%i): %s", i_kp_, utility_.toString(path_.points.at(i_kp_)).c_str());
    ////////ROS_INFO("prevKP: %s", utility_.toString(prevKP_).c_str());
    ////////ROS_INFO("theta: %f", theta);


    double x_dot, y_dot;
    if(path_.points.at(i_kp_).motionState.velocities.size() > 0 &&
        (fabs(path_.points.at(i_kp_).motionState.velocities.at(0)) > 0.01) )
    {
      ////////////ROS_INFO("x_dot and y_dot = specified velocities");
      x_dot = fabs(path_.points.at(i_kp_).motionState.velocities.at(0));
      y_dot = fabs(path_.points.at(i_kp_).motionState.velocities.at(1));
    }
    else
    {
      ////////////ROS_INFO("calculating x_dot and y_dot");
      x_dot = fabs(MAX_SPEED * cos(theta));
      y_dot = x_dot*tan(theta);
    }
    ////////ROS_INFO("x_dot: %f y_dot: %f", x_dot, y_dot);

    if(path_.points.at(i_kp_).motionState.velocities.size() == 0)
    {
      path_.points.at(i_kp_).motionState.velocities.push_back(x_dot);
      path_.points.at(i_kp_).motionState.velocities.push_back(y_dot);
      path_.points.at(i_kp_).motionState.velocities.push_back(0);
    }
  
    double y_diff = path_.points.at(i_kp_).motionState.positions.at(1) - prevKP_.positions.at(1);
    double x_diff = path_.points.at(i_kp_).motionState.positions.at(0) - prevKP_.positions.at(0);
    bool x_diff_greater = fabs(x_diff) > fabs(y_diff);
    
    ////////ROS_INFO("x_dot: %f y_dot: %f x_diff: %f y_diff: %f x_diff_greater: %s", x_dot, y_dot, x_diff, y_diff, x_diff_greater ? "True" : "False");

    // *** Set the new target ***
    if(x_diff_greater)
      setMaxV(x_dot);
    else
      setMaxV(y_dot);
    setTarget(path_.points.at(i_kp_).motionState);
    ////////ROS_INFO("After setting new target:");
    ////////ROS_INFO("Prev KP: %s", utility_.toString(prevKP_).c_str());
    ////////ROS_INFO("Target: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());





    /** Bezier */
    // If its a Bezier curve traj, and we're at a Bezier point
    // all points between first and last are bezier point
    //std::cout<<"\nc: "<<c<<" i_cs.size(): "<<i_cs.size()<<"\n";
    //std::cout<<"\ncurves.size(): "<<curves.size()<<"\n";
    if( (c < i_cs.size() && path_.points.size() > 2 && i_kp_ == i_cs.at(c)+1))
    {
      ////////ROS_INFO("At Bezier Curve %i", c);
      //////////////////ROS_INFO("timeFromStart_: %f", timeFromStart_.toSec());
      //std::cout<<"\ncurves.at("<<(int)c<<").size(): "<<curves.at(c).points_.size();

      // Insert all points on the curves into the trajectory
      // TODO: Why am I not pushing on every point of the curve?
      for(uint32_t p=1;p<curves.at(c).points_.size();p++) 
      {
        insertPoint(curves.at(c).points_.at(p), res);

        // If it's the first or last point on the curve, 
        // push the index to knot point vector
        // Only add the first point on 1st curve - other curves will
        // have their first points as the last point from reflexxes straight-line
        if(p==curves.at(c).points_.size()-1) //||
            //(c == i_cs.at(0) && p == 1) ) 
        {
          ////////////ROS_INFO("Pushing on i_knotpoint: %i", (int)res.trajectory.trajectory.points.size()-1);
          res.trajectory.i_knotPoints.push_back(
                          res.trajectory.trajectory.points.size() - 1 );
        } // end if knot point
      } // end for


      // Create a BezierInfo for the curve to return with the trajec
      ramp_msgs::BezierCurve bi;
      bi.segmentPoints  = curves.at(c).segmentPoints_;
      bi.controlPoints  = curves.at(c).controlPoints_;
      bi.ms_maxVA       = curves.at(c).ms_max_;
      //bi.ms_initialVA   = curves.at(c).ms_init_;
      bi.u_0            = req_.bezierCurves.at(c).u_0;
      bi.u_dot_0        = curves.at(c).u_dot_0_;
      bi.l              = curves.at(c).l_;
      bi.u_target       = curves.at(c).u_target_;
      bi.points         = curves.at(c).points_;
      bi.u_values       = curves.at(c).u_values_;
      res.trajectory.curves.push_back(bi);


      c++;
    } // end if bezier


    /** Straight Line Segment */
    // Else if straight-line segment
    else 
    {
      ////////ROS_INFO("In else, straight-line segment");

      // Get rotation if needed
      double trajec_size = res.trajectory.trajectory.points.size();

      trajectory_msgs::JointTrajectoryPoint last = 
            res.trajectory.trajectory.points.at(trajec_size-1);

      trajectory_msgs::JointTrajectoryPoint next_knot =
            utility_.getTrajectoryPoint(path_.points.at(i_kp_).motionState);

      //////////ROS_INFO("=== Orientation Information ===");
      //////////ROS_INFO("last: %s", utility_.toString(last).c_str());
      //////////ROS_INFO("next_knot: %s", utility_.toString(next_knot).c_str());
      //////////ROS_INFO("utility_.findAngleFromAToB(last, next_knot): %f", utility_.findAngleFromAToB(last, next_knot));
      //////////ROS_INFO("utility_.findDistanceBetweenAngles(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot)): %f", utility_.findDistanceBetweenAngles(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot)));


      // Check for goal because the robot should not rotate
      // if it overshoots the goal.
      if(!checkTarget() || type_ == HOLONOMIC) 
      {

        // Set orientation threshold that requires a rotation 
        // before continuing to the next knot point
        double threshold = 0.3f; 
        ////////////////ROS_INFO("threshold: %f", threshold);

        // If we need to rotate towards the next knot point
        // 0.0872664 = 5 degrees
        if(fabs(utility_.findDistanceBetweenAngles(last.positions.at(2), 
                utility_.findAngleFromAToB(last, next_knot))) > threshold) 
        {
          ////////ROS_INFO("Calling rotate");
          ////////ROS_INFO("c: %i i_cs.size(): %i", c, (int)i_cs.size());
          /*std::vector<trajectory_msgs::JointTrajectoryPoint> rotate_points = 
            rotate(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot),
                    last.velocities.at(2), last.accelerations.at(2));*/

          // If we have reached the end of the non-holonomic segment, 
          // stop generating points
          //if(c >= i_cs.size())
          if(i_kp_ > 1 && type_ == HYBRID)
          {
            break;
          }

          rotateOOP(last.positions[2], utility_.findAngleFromAToB(last, next_knot),
                    last.velocities.at(2), last.accelerations.at(2), res.trajectory.trajectory.points);


          setSelectionVector();
          reflexxesData_.resultValue = 0;
          prevKP_ = res.trajectory.trajectory.points.at(
              res.trajectory.trajectory.points.size() - 1);
        } // end if rotate
        /*else {
          //////////////////ROS_INFO("No rotation needed");
        }*/
      } // end if final state is not already reached
      /*else {
        //////////////////ROS_INFO("Check goal returns true");
      }*/

      setTarget(path_.points.at(i_kp_).motionState);
      ////////////ROS_INFO("Prev KP: %s", utility_.toString(prevKP_).c_str());
      ////////////ROS_INFO("Target: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());

      // Check they are not the same point
      if(utility_.positionDistance(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1).positions, 
            path_.points.at(i_kp_).motionState.positions) > 0.0001)
      {
        ////////////ROS_INFO("Pushing on points b/c dist: %f", utility_.positionDistance(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1).positions, path_.points.at(i_kp_).motionState.positions));
              
        size_t t_size = res.trajectory.trajectory.points.size();
        // We go to the next knotpoint only once we reach this one
        while (!finalStateReached()) 
        {

          trajectory_msgs::JointTrajectoryPoint p = spinOnce();
          ////////////ROS_INFO("p: %s", utility_.toString(p).c_str());
          ////////////ROS_INFO("result: %i", reflexxesData_.resultValue);
          if(reflexxesData_.resultValue == -100)
          {
            //////ROS_ERROR("An error occurred in Reflexxes, setting res.error=1 and returning");
            res.error = true;
            return false;
          }

          // Compute the motion state at t+1 and save it in the trajectory
          res.trajectory.trajectory.points.push_back(p);
        } // end while

       
        if(res.trajectory.trajectory.points.size() > t_size)
        {
          ////////////ROS_INFO("Pushing on i_knotpoint: %i", (int)res.trajectory.trajectory.points.size()-1);
          res.trajectory.i_knotPoints.push_back(res.trajectory.trajectory.points.size() - 1);
        }
      } // end if different points

      // Else if there's only 2 points and the current point and next knot point are the same
      // It was a path with the same point
      else if(req.path.points.size() == 2) 
      {
        ////////////ROS_INFO("Last position and next knot point are the same position, path size == 2");
        res.trajectory.trajectory.points.push_back(res.trajectory.trajectory.points.at(0));
      }
      //else {
        //////////////////ROS_INFO("Last position and next knot point are the same position, path size > 2");
      //}
    } // end if

    ////////////ROS_INFO("Outside of while");
    // Set previous knot point
    prevKP_ = res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size() - 1);
    //////////////ROS_INFO("After setting new prevKP");


    // Check if Reflexxes overshot target
    /*if(!lastPointClosest(res.trajectory)) 
    {
      //////////////ROS_INFO("Last point is not closest");

      res.trajectory.trajectory.points.pop_back();
      res.trajectory.i_knotPoints.at(res.trajectory.i_knotPoints.size()-1) =
       res.trajectory.trajectory.points.size()-1; 

      timeFromStart_ -= ros::Duration(CYCLE_TIME_IN_SECONDS);
      
      // If it's the first kp and there's no curve
      if(i_kp_ == 1 && req.path.points.size() > 2 && type_ != HYBRID)
      {
        //////////////////ROS_INFO("Remvoing last knot point index");
        res.trajectory.i_knotPoints.pop_back();
      }
    } // end if checking Reflexxes overshooting*/
    //////////////ROS_INFO("Past last point closest");

      //////////////////ROS_INFO("Reached target: %s \nAt state: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str(), utility_.toString(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1)).c_str());
  } // end for each knot point (outer-most loop)
 
  //////////////ROS_INFO("Outside of for");
  // Check that the last point is a knot point
  if(res.trajectory.trajectory.points.size()-1 != res.trajectory.i_knotPoints.at(res.trajectory.i_knotPoints.size()-1))
  {
    ////////////ROS_INFO("Pushing on i_knotpoint: %i", (int)res.trajectory.trajectory.points.size()-1);
    res.trajectory.i_knotPoints.push_back(res.trajectory.trajectory.points.size()-1);
  }

  //////////////ROS_INFO("Exiting trajectoryRequest");
  return true;
} // End trajectoryRequest callback



/** This performs a rotation using Reflexxes */
const std::vector<trajectory_msgs::JointTrajectoryPoint> MobileBase::rotate(const double start, const double goal, const double start_v, const double start_a) {
  ////////////////ROS_INFO("In MobileBase::rotate");
  ////////////////ROS_INFO("start: %f goal: %f start_v: %f start_a: %f", start, goal, start_v, start_a);
  std::vector<trajectory_msgs::JointTrajectoryPoint> result;

  double targetTheta = utility_.findDistanceBetweenAngles(start, goal);

  // Set Selection Vector up for rotation
  setSelectionVectorRotation();

  // Set current velocity and acceleration values for x to 0
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0;

  // Set current values for orientation
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[1] = start;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = start_v;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = start_a;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[1] = goal;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[1] = 0;
  
  // Call Reflexxes until goal is reached
  reflexxesData_.resultValue = 0;
  while(!finalStateReached()) 
  {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce();
    ////////////////ROS_INFO("p: %s", utility_.toString(p).c_str());
    result.push_back(p);
  }
  
  if(result.size() > 0) 
  {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = 
      result.at(result.size()-1).positions.at(2);
  }


  ////////////////ROS_INFO("Exiting rotate");
  return result;
} // End rotate



void MobileBase::rotateOOP(const double start, const double goal, const double start_v, const double start_a, std::vector<trajectory_msgs::JointTrajectoryPoint>& result)
{
  ////////////////ROS_INFO("In MobileBase::rotate");
  ////////////////ROS_INFO("start: %f goal: %f start_v: %f start_a: %f", start, goal, start_v, start_a);

  double targetTheta = utility_.findDistanceBetweenAngles(start, goal);

  // Set Selection Vector up for rotation
  setSelectionVectorRotation();

  // Set current velocity and acceleration values for x to 0
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0;

  // Set current values for orientation
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[1] = start;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = start_v;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = start_a;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[1] = goal;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[1] = 0;
  
  // Call Reflexxes until goal is reached
  reflexxesData_.resultValue = 0;
  while(!finalStateReached()) 
  {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce();
    ////////////////ROS_INFO("p: %s", utility_.toString(p).c_str());
    result.push_back(p);
  }
  
  if(result.size() > 0) 
  {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = 
      result.at(result.size()-1).positions.at(2);
  }


  ////////////////ROS_INFO("Exiting rotate");
}


const std::vector<trajectory_msgs::JointTrajectoryPoint> MobileBase::verticalLine(ramp_msgs::MotionState start, 
    ramp_msgs::MotionState goal)
{
  std::vector<trajectory_msgs::JointTrajectoryPoint> result;
  double start_y = start.positions.at(1);
  double start_ydot = start.velocities.at(1);
  double start_yddot = start.accelerations.at(1);
  double goal_y  = goal.positions.at(1);
  double goal_ydot = goal.velocities.at(1);
  
  // Set selection to use x
  setSelectionVector();

  // Set current velocity and acceleration values for theta to 0
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = 0;

  // Set current values for y
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = start_y;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = start_ydot;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = start_yddot;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = goal_y;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = goal_ydot;

  // Call Reflexxes until goal is reached
  reflexxesData_.resultValue = 0;
  while(!finalStateReached()) 
  {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce(true);
    ////////////////ROS_INFO("p: %s", utility_.toString(p).c_str());
    result.push_back(p);
  }
 
  // Restore reflexxes data
  if(result.size() > 0) 
  {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = 
      result.at(result.size()-1).positions.at(0);
    reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = 
      result.at(result.size()-1).velocities.at(0);
    reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 
      result.at(result.size()-1).accelerations.at(0);
  }


  ////////////////ROS_INFO("Exiting verticalLine");
  return result;
}


const bool MobileBase::checkTarget() {
  std::vector<double> p_current, p_target, v_current, v_target;
  
  for(uint8_t i=0;i<3;i++) {
    p_current.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
    p_target.push_back( reflexxesData_.inputParameters->TargetPositionVector->VecData[i] );
    
    v_current.push_back(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
    v_target.push_back( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] );
  }

  if(fabs(utility_.positionDistance(p_current, p_target)) < 0.0001 &&
      fabs(utility_.positionDistance(v_current, v_target)) < 0.0001 ) 
  {
    return true; 
  }

  return false;
}


const bool MobileBase::lastPointClosest(const ramp_msgs::RampTrajectory& traj) const {
  //////////////////ROS_INFO("In lastPointClosest");
  
  std::vector<double> target_p, target_v; 
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[1]);
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[2]);

  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]);
  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[1]);
  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[2]);

  //////////////////ROS_INFO("After setting targets");

  //////////////////ROS_INFO("traj size: %i", (int)traj.trajectory.points.size());
  trajectory_msgs::JointTrajectoryPoint last = traj.trajectory.points.at(traj.trajectory.points.size()-1);
  trajectory_msgs::JointTrajectoryPoint nextToLast_to_last = 
    traj.trajectory.points.at(traj.trajectory.points.size()-2);

  //////////////////ROS_INFO("last: %s \nnext_to_last: %s", utility_.toString(last).c_str(), utility_.toString(nextToLast_to_last).c_str());

  //////////////////ROS_INFO("Done setting last and nextToLast_to_last");
  //////////////////ROS_INFO("last.positions size(): %i target_p.size: %i", (int)last.positions.size(), (int)target_p.size());
 
  double dist_last_p = utility_.getEuclideanDist(last.positions, target_p);
  double dist_nextToLast_p = utility_.getEuclideanDist(nextToLast_to_last.positions, target_p);

  //////////////////ROS_INFO("dist_last_p: %f dist_nextToLast_p: %f", dist_last_p, dist_nextToLast_p);
 
  double dist_last_v = utility_.getEuclideanDist(last.velocities, target_v);
  double dist_nextToLast_v = utility_.getEuclideanDist(nextToLast_to_last.velocities, target_v);
  
  //////////////////ROS_INFO("dist_last_v: %f dist_nextToLast_v: %f", dist_last_v, dist_nextToLast_v);

  double dist_last = dist_last_p + dist_last_v;
  double dist_nextToLast = dist_nextToLast_p + dist_nextToLast_v;
  
  //////////////////ROS_INFO("dist_last: %f dist_nextToLast: %f fabs(dist_last - dist_nextToLast): %f", dist_last, dist_nextToLast, fabs(dist_last - dist_nextToLast));

  //////////////////ROS_INFO("Exiting MobileBase::lastPointClosest");
  return (dist_last < dist_nextToLast) && fabs(dist_last - dist_nextToLast) > 0.0001;
}


// Returns true if the target has been reached
bool MobileBase::finalStateReached() const 
{
  
  if(timeFromStart_ >= timeCutoff_) 
  {
    ////////ROS_WARN("timeFromStart_ > timeCutoff_ (%f)", timeCutoff_.toSec());
    ////////ROS_WARN("Check this trajectory request");
    ////////ROS_WARN("reflexxesData_.resultValue: %i", reflexxesData_.resultValue);
    ////////ROS_WARN("Request: %s", utility_.toString(req_).c_str());
  }

  int i = reflexxesData_.inputParameters->SelectionVector->VecData[1] == 1;
  bool position_goal_met = fabs( reflexxesData_.inputParameters->TargetPositionVector->VecData[i]- reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] ) < 0.01;
  bool velocity_goal_met = fabs( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i]- reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] ) < 0.01;
  bool goal_reached = position_goal_met && velocity_goal_met;

  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED)  
      || goal_reached ||  (timeFromStart_ >= timeCutoff_);
} // End finalStateReached



