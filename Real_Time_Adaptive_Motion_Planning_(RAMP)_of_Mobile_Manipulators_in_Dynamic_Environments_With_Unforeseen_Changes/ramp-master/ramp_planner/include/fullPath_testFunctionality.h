/* 
 * File:   fullPath_testFunctionality.h
 * Author: annex01lt
 *
 * Created on October 1, 2016, 5:41 PM
 */

#ifndef FULLPATH_TESTFUNCTIONALITY_H
#define	FULLPATH_TESTFUNCTIONALITY_H

// include google test library
#include <gtest/gtest.h>

// include main file of the server process (Trajectory evaluation) as header file.
#include "ros/ros.h"
#include "range.h"
#include "utility.h"


// To include these, you have to link all the .cpp files in these modules in the ramp_planner CMakeLists file
// We don't need to setup and start these services in this file 
// They can be started before running this test file
//#include "../../trajectory_evaluation/include/main_testing.h"
//#include "../../trajectory_generator/include/main_testing.h"

// include messages that needed for client process (planner).
#include "ramp_msgs/MotionState.h"
#include "knot_point.h"
#include "ramp_msgs/Path.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Population.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
#include "ramp_msgs/ModificationRequest.h"

#include "ramp_msgs/EvaluationSrv.h"
#include "ramp_msgs/TrajectorySrv.h"


class fullPathFixtureTest:public ::testing::Test{
    public:
        // Constructor & Destructor 
        fullPathFixtureTest();
        virtual ~fullPathFixtureTest();
    
        // Setup & TearDowm
        virtual void SetUp();        
        virtual void TearDown();

        //Callback method that calls the callback function of the server process (Trajectory evaluation).
        //bool Callback(ramp_msgs::EvaluationSrv::Request& reqs, ramp_msgs::EvaluationSrv::Response& resps);

        // trajectoryBuilder method builds a trajectory with number of Joint Points
        ramp_msgs::RampTrajectory trajectoryBulider(int numJoints);
        
        // Data Members.
        ros::NodeHandle handle;
        ros::ServiceClient _client;
        
        // Arguments for Evaluation Request. 
        ramp_msgs::EvaluationSrv _evaluationSrv;
        ramp_msgs::EvaluationRequest _evaluationRequest;
        
};

fullPathFixtureTest::fullPathFixtureTest(){}

fullPathFixtureTest::~fullPathFixtureTest(){}

/*
 * Initialize clients
 * Servers should be started
 */
void fullPathFixtureTest::SetUp(){
  // Initialize clients.
    
  _client = handle.serviceClient<ramp_msgs::EvaluationSrv>("trajectory_evaluation");
  //_service = handle.advertiseService("trajectory_evaluation", &fullPathFixtureTest::Callback,this);
}        

void fullPathFixtureTest::TearDown(){}




/*bool fullPathFixtureTest::Callback(ramp_msgs::EvaluationSrv::Request& reqs,
                                               ramp_msgs::EvaluationSrv::Response& resps){
//    if(handleRequest(reqs,resps)){
//        return true;
//    }else{
//        return false;
//    }
}*/

ramp_msgs::RampTrajectory fullPathFixtureTest::trajectoryBulider(int numJointPoints){

    // A trajectory needed for trajectory request
    ramp_msgs::RampTrajectory result;
    trajectory_msgs::JointTrajectory _jointTrajectory;

    
    // Ranges of positions and velocities 
    Range x(0, 3.5);
    Range y(0, 3.5);
    Range relative_direction(0, 0.33);

    Range v_x(0, 0.33);
    Range v_y(0, 0.33);
    Range w(0,  PI/2.f);
    //------------------------------------------------------
    
    // Seed Joint points and push them into trajectory
    for(int i=0 ; i < numJointPoints ; i++)
    {  
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(x.random());
    _jointTrajectoryPoint.positions.push_back(y.random());
    _jointTrajectoryPoint.positions.push_back(relative_direction.random());
    
    _jointTrajectoryPoint.velocities.push_back(v_x.random());
    _jointTrajectoryPoint.velocities.push_back(v_y.random());
    _jointTrajectoryPoint.velocities.push_back(w.random());
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    //------------------------------------------------------
    
    // Initialize the trajectory's arguments for evaluation request 
    result.trajectory = _jointTrajectory;
    result.id = 1;
    result.feasible = true;
    result.fitness = -1;  
    result.t_firstCollision = ros::Duration(9999.f);
    result.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------
    return result;
}

#endif	/* FULLPATH_TESTFUNCTIONALITY_H */

