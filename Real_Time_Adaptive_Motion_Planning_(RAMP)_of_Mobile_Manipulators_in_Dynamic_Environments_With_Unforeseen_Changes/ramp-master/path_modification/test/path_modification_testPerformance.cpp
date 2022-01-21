/* 
 * File:   path_modification_testPerformance.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:12 PM
 */


// include header file of the fixture tests
#include "path_modification_fixtureTest.h"

TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_50ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.050).toSec(), _duration)
                    <<"The path modification service spun more than 50 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_45ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.045).toSec(), _duration)
                    <<"The path modification service spun more than 45 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_40ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.040).toSec(), _duration)
                    <<"The path modification service spun more than 40 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_35ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.035).toSec(), _duration)
                    <<"The path modification service spun more than 35 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_30ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.030).toSec(), _duration)
                    <<"The path modification service spun more than 30 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_25ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.025).toSec(), _duration)
                    <<"The path modification service spun more than 25 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_20ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.020).toSec(), _duration)
                    <<"The path modification service spun more than 20 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_15ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.015).toSec(), _duration)
                    <<"The path modification service spun more than 15 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_10ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.010).toSec(), _duration)
                    <<"The path modification service spun more than 10 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each_In_05ms){
    
    // A vector of paths needed for trajectory request
    std::vector<ramp_msgs::Path> _paths;        

    // --------------- Seed Two Paths ---------------------
   for(unsigned int i=0 ; i<2 ; i++) {
    ramp_msgs::Path _path_;
    
    // Seed knot points and push them into path
    { // Scop the variables & Seed the first knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    { // Scop the variables & Seed the second knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.75f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((-1.f*PI/2.f));
    knotPoint.motionState.velocities.push_back(-0.20f);
    knotPoint.motionState.velocities.push_back(-0.15f);
    knotPoint.motionState.velocities.push_back(-0.13f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------

    { // Scop the variables & Seed the third knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(3.25f);
    knotPoint.motionState.positions.push_back(0.f);
    knotPoint.motionState.positions.push_back((3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.velocities.push_back(0.f);
    knotPoint.motionState.time = 0;    
    _path_.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "crossover";
    // -----------------------------------------------------
    
    try{          

        // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a modification request
          double _duration = (_endTime - _startTime);
          // Expected modification request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.005).toSec(), _duration)
                    <<"The path modification service spun more than 5 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call path modification service.";
    }
    
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "path_modification_testPerfomance_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;

}

