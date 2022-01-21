/* 
 * File:   trajectory_generator_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 9, 2015, 1:24 PM
 */


#ifndef TRAJECTORY_GENERATOR_FIXTURETEST_H
#define	TRAJECTORY_GENERATOR_FIXTURETEST_H

// include google test library
#include <gtest/gtest.h>

// include main file of the server process (trajectory generator) as header file.
#include "main_testing.h"

// include messages that needed for client process (planner).
/*#include "ramp_msgs/Path.h"
#include "ramp_msgs/KnotPoint.h"
#include "ramp_msgs/MotionState.h"
#include "ramp_msgs/BezierCurve.h"*/

#include "utility.h"

class trajectoryGeneratorFixtureTest:public ::testing::Test{
    public:
        // Constructor & Destructor. 
        trajectoryGeneratorFixtureTest();
        virtual ~trajectoryGeneratorFixtureTest();
    
        // Setup & TearDowm.
        virtual void SetUp();        
        virtual void TearDown();
        
        //Callback method that calls the callback function of the server process (trajectory generator).
        bool Callback(ramp_msgs::TrajectorySrv::Request& req, ramp_msgs::TrajectorySrv::Response& res);

        // Data Members.
        ros::NodeHandle client_handle, server_handle;
        ros::ServiceClient _client;
        ros::ServiceServer _service;
        
        // Argument for Trajectory Request. 
        ramp_msgs::TrajectorySrv _trajectorySrv;

};

// Constructor: Initialize start and final points  
trajectoryGeneratorFixtureTest::trajectoryGeneratorFixtureTest(){}

trajectoryGeneratorFixtureTest::~trajectoryGeneratorFixtureTest(){}

void trajectoryGeneratorFixtureTest::SetUp(){
  // Initialize client and server processes.  
  _client = client_handle.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");
  _service = server_handle.advertiseService("/trajectory_generator", &trajectoryGeneratorFixtureTest::Callback,this);
}        

void trajectoryGeneratorFixtureTest::TearDown(){}

bool trajectoryGeneratorFixtureTest::Callback(ramp_msgs::TrajectorySrv::Request& req, ramp_msgs::TrajectorySrv::Response& res){
    if(requestCallback(req,res)){
        return true;
    }else{
        return false;
    }   
}


#endif	/* TRAJECTORY_GENERATOR_FIXTURETEST_H */

