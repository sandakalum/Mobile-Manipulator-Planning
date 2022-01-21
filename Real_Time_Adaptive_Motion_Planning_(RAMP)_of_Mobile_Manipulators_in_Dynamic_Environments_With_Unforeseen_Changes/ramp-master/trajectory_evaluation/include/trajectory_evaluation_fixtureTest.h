/* 
 * File:  trajectory_evaluation_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:03 PM
 */


#ifndef TRAJECTORY_EVALUATION_FIXTURETEST_H
#define	TRAJECTORY_EVALUATION_FIXTURETEST_H

// include google test library
#include <gtest/gtest.h>

// include main file of the server process (Trajectory evaluation) as header file.
#include "main_testing.h"

// include messages that needed for client process (planner).
/*#include "ramp_msgs/MotionState.h"
#include "ramp_msgs/EvaluationRequest.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"*/

#include "utility.h"


class trajectoryEvaluationFixtureTest:public ::testing::Test{
    public:
        // Constructor & Destructor 
        trajectoryEvaluationFixtureTest();
        virtual ~trajectoryEvaluationFixtureTest();
    
        // Setup & TearDowm
        virtual void SetUp();        
        virtual void TearDown();

        //Callback method that calls the callback function of the server process (Trajectory evaluation).
        bool Callback(ramp_msgs::EvaluationSrv::Request& req, ramp_msgs::EvaluationSrv::Response& res);

        // Data Members.
        ros::NodeHandle client_handle, server_handle;
        ros::ServiceClient _client;
        ros::ServiceServer _service;
        
        // Arguments for Evaluation Request. 
        ramp_msgs::EvaluationSrv _evaluationSrv;
        
};

trajectoryEvaluationFixtureTest::trajectoryEvaluationFixtureTest(){}

trajectoryEvaluationFixtureTest::~trajectoryEvaluationFixtureTest(){}

void trajectoryEvaluationFixtureTest::SetUp(){
  // Initialize client and server processes.  
  _client = client_handle.serviceClient<ramp_msgs::EvaluationSrv>("/trajectory_evaluation");
  _service = server_handle.advertiseService("/trajectory_evaluation", &trajectoryEvaluationFixtureTest::Callback,this);
}        

void trajectoryEvaluationFixtureTest::TearDown(){}

bool trajectoryEvaluationFixtureTest::Callback(ramp_msgs::EvaluationSrv::Request& req, ramp_msgs::EvaluationSrv::Response& res){
    if(handleRequest(req,res)){
        return true;
    }else{
        return false;
    }
}

#endif	/* TRAJECTORY_EVALUATION_FIXTURETEST_H */

