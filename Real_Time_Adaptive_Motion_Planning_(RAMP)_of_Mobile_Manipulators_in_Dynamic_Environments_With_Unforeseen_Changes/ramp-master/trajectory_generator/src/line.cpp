#include "line.h"



Line::Line() {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
  timeCutoff_ = ros::Duration(3.5);
}

Line::~Line() {
  if(reflexxesData_.rml != 0) {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }
  if(reflexxesData_.inputParameters) {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  if(reflexxesData_.outputParameters != 0) {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}


const ramp_msgs::MotionState Line::buildMotionState(const ReflexxesData data) 
{
  ramp_msgs::MotionState result;

  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      
    // If Reflexxes has not been called yet
    if(data.outputParameters->NewPositionVector->VecData[0] == -99) {
      result.positions.push_back(data.inputParameters->CurrentPositionVector->VecData[i]);
      result.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[i]);
      result.accelerations.push_back(data.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
    
    // If selection vector is true
    else if(reflexxesData_.inputParameters->SelectionVector->VecData[i]) {
      result.positions.push_back(data.outputParameters->NewPositionVector->VecData[i]);
      result.velocities.push_back(data.outputParameters->NewVelocityVector->VecData[i]);
      result.accelerations.push_back(data.outputParameters->NewAccelerationVector->VecData[i]);

      /*if(i == 2) {
        result.positions.at(2) = utility_.displaceAngle(prevKP_.positions.at(2), 
                                  data.outputParameters->NewPositionVector->VecData[i]);
      }*/
    } // end if selection vector is true
    
    // Else if we're at orientation dof
    /*else if(i == 2) {
      
      // If straight-line paths, make theta be towards next knot result
      double theta = utility_.findAngleFromAToB( prevKP_.positions.at(0),
                                                 prevKP_.positions.at(1),
                                  data.inputParameters->TargetPositionVector->VecData[0],
                                  data.inputParameters->TargetPositionVector->VecData[1]);

      // Get angular velocity
      double w = (theta - data.inputParameters->CurrentPositionVector->VecData[2]) / 
                  CYCLE_TIME_IN_SECONDS;

      // Push on p,v,a
      // TODO: Manually keep acceleration?
      result.positions.push_back(theta);
      result.velocities.push_back(w);
      result.accelerations.push_back(0);

      // Set values in Reflexxes
      // TODO: Necessary?
      reflexxesData_.outputParameters->NewPositionVector->VecData[2] = theta;
      reflexxesData_.outputParameters->NewVelocityVector->VecData[2] = w;
    } // end else-if orientation*/
      
    // Else, just push on the current value
    else {
      result.positions.push_back(
          reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
      result.velocities.push_back(
          reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
      result.accelerations.push_back(
          reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  }

  result.time = timeFromStart_.toSec();
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  //std::cout<<"\nresult: "<<utility_.toString(result)<<"\n";
  return result;
}



const bool Line::finalStateReached() 
{


  bool position_goal_met = fabs( reflexxesData_.inputParameters->TargetPositionVector->VecData[0]- reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] ) < 0.01;
  bool velocity_goal_met = fabs( reflexxesData_.inputParameters->TargetVelocityVector->VecData[1]- reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] ) < 0.01;
  bool goal_reached = position_goal_met && velocity_goal_met;

  //return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED || goal_reached || 
      (timeFromStart_ >= timeCutoff_));
}

  
const std::vector<ramp_msgs::MotionState> Line::generatePoints() {
  std::vector<ramp_msgs::MotionState> result;

  result.push_back(start_);
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  reflexxesData_.resultValue = 0;

  while(!finalStateReached()) {
    result.push_back(spinOnce()); 
  }

  return result;
}



void Line::init(const ramp_msgs::MotionState start, 
                const ramp_msgs::MotionState goal) 
{
  //std::cout<<"\nIn init, start: "<<utility_.toString(start);
  start_  = start;
  goal_   = goal;
  
  timeCutoff_ = ros::Duration(35);

  initReflexxes();
  setReflexxesCurrent();
  setReflexxesTarget();
  setReflexxesSelection();
}

void Line::initReflexxes() {
  // Set DOF
  reflexxesData_.NUMBER_OF_DOFS = 3;

  // Initialize all relevant objects of the Type II Reflexxes Motion Library
  if(reflexxesData_.rml == 0) {
    reflexxesData_.rml = new ReflexxesAPI( 
            reflexxesData_.NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );

    reflexxesData_.inputParameters = new RMLPositionInputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );

    reflexxesData_.outputParameters = new RMLPositionOutputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );
  } // end if

  // Use time synchronization so the robot drives in a straight line towards goal 
  reflexxesData_.flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = fabs(start_.velocities.at(0));
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = fabs(start_.velocities.at(1));
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[2] = fabs(start_.velocities.at(2));
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 0.66;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = 0.66;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[2] = PI/4;
  

  // As the maximum jerk values are not known, this is just to try
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[1] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[2] = PI/3;

  // Set flag value to know if Reflexxes has been called yet
  reflexxesData_.outputParameters->NewPositionVector->VecData[0] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[1] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[2] = -99;
  
  // Result
  reflexxesData_.resultValue = 0;
}


// Initialize variables just after receiving a service request
// TODO: Check if start_ has been set
void Line::setReflexxesCurrent() {
  
  // Initialise the time to use for each trajectory point
  timeFromStart_ = ros::Duration(0);
  
  
  // Set the positions of the robot as Reflexxes input
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = 
      start_.positions.at(i);
  }

  // Set the current velocities of the robot as Reflexxes input
  if(start_.velocities.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = 
        start_.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations of the robot as Reflexxes input
  if(start_.accelerations.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = 
        start_.accelerations.at(i);
    }
  }
  else {//log some error
  }
} // End setReflexxesCurrent




// TODO: Check if goal has been set
void Line::setReflexxesTarget() {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    // Position
    reflexxesData_.inputParameters->TargetPositionVector->VecData[i] =
     goal_.positions.at(i);
    
    // Velocity, if specified
    if(goal_.velocities.size() > 0) {
      if(goal_.velocities.at(i) == 0) {
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] =
            0.00000000000000001;
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[i] =
            0.00000000000000001;
      }
      else {
        reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] =
            goal_.velocities.at(i);
      }
    }
  } // end for 

} // End setReflexxesTarget


void Line::setReflexxesSelection() {
  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = false;
}


const ramp_msgs::MotionState Line::spinOnce() {
  // Calling the Reflexxes OTG algorithm
  reflexxesData_.resultValue = 
    reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, 
                                    reflexxesData_.outputParameters, 
                                    reflexxesData_.flags);


  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  ramp_msgs::MotionState result = buildMotionState(reflexxesData_);


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = 
      *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
      *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
      *reflexxesData_.outputParameters->NewAccelerationVector;

  return result;
}
