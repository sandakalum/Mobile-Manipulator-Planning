/* 
 * File:   path_modification_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:12 PM
 */


#ifndef PATH_MODIFICATION_FIXTURETEST_H
#define	PATH_MODIFICATION_FIXTURETEST_H

// include google test library
#include <gtest/gtest.h>

// include main file of the server process (path modification) as header file.
#include "../include/main_testing.h"

// include messages that needed for client process (planner).
#include "ramp_msgs/MotionState.h"
#include "ramp_msgs/KnotPoint.h"
#include "ramp_msgs/Path.h"
#include "ramp_msgs/ModificationRequest.h"


class pathModificationFixtureTest:public ::testing::Test{
    public:
        // Constructor & Destructor 
        pathModificationFixtureTest();
        virtual ~pathModificationFixtureTest();
    
        // Setup & TearDowm
        virtual void SetUp();        
        virtual void TearDown();

        //Callback method that calls the callback function of the server process (path modification).
        bool Callback(ramp_msgs::ModificationRequest::Request& req, ramp_msgs::ModificationRequest::Response& res);

        // Data Members.
        ros::NodeHandle client_handle, server_handle;
        ros::ServiceClient _client;
        ros::ServiceServer _service;
        
        // Argument for Modification Request. 
        ramp_msgs::ModificationRequest _modificationRequest;
        
};

pathModificationFixtureTest::pathModificationFixtureTest(){}

pathModificationFixtureTest::~pathModificationFixtureTest(){}

void pathModificationFixtureTest::SetUp(){
  // Initialize client and server processes.  
  _client = client_handle.serviceClient<ramp_msgs::ModificationRequest>("/path_modification");
  _service = server_handle.advertiseService("/path_modification", &pathModificationFixtureTest::Callback,this);
}        

void pathModificationFixtureTest::TearDown(){}

bool pathModificationFixtureTest::Callback(ramp_msgs::ModificationRequest::Request& req, ramp_msgs::ModificationRequest::Response& res){
    if(handleRequest(req,res)){
        return true;
    }else{
        return false;
    }
}


#endif	/* PATH_MODIFICATION_FIXTURETEST_H */

