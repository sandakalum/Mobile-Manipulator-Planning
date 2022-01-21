/* 
 * File:   path_modification_testFunctionality.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:12 PM
 */


// include header file of the fixture tests
#include "path_modification_fixtureTest.h"


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Two_KnotPoints_Of_Each){
    
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
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
       
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(3 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Three_KnotPoints_Of_Each){
    
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
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Four_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    
}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Five_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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
    _modificationRequest.request.op = "swap";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Sixth_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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

    { // Scop the variables & Seed the sixth knot point 
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
    _modificationRequest.request.op = "delete";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Seven_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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

    { // Scop the variables & Seed the sixth knot point 
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

    { // Scop the variables & Seed the seventh knot point 
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
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Eight_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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

    { // Scop the variables & Seed the sixth knot point 
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

    { // Scop the variables & Seed the seventh knot point 
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

    { // Scop the variables & Seed the eight knot point 
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
    
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "delete";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Nine_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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

    { // Scop the variables & Seed the sixth knot point 
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

    { // Scop the variables & Seed the seventh knot point 
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

    { // Scop the variables & Seed the eight knot point 
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

    { // Scop the variables & Seed the ninth knot point 
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
    
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(4 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


TEST_F(pathModificationFixtureTest, testPath_Modification_Two_Paths_With_Ten_KnotPoints_Of_Each){
    
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

    { // Scop the variables & Seed the forth knot point 
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

    { // Scop the variables & Seed the fifth knot point 
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

    { // Scop the variables & Seed the sixth knot point 
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

    { // Scop the variables & Seed the seventh knot point 
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

    { // Scop the variables & Seed the eight knot point 
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

    { // Scop the variables & Seed the ninth knot point 
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

    { // Scop the variables & Seed the tenth knot point 
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
    
    // ------- Push the path into modification request --------
    _paths.push_back(_path_); 

   } // end of scop of the for-loop
   
    
    // Select an operation for modification request  -------
    _modificationRequest.request.paths = _paths;   
    _modificationRequest.request.op = "insert";
    // -----------------------------------------------------
    
    try{          
          // Request a modification
          _client.call(_modificationRequest);
          
          // Expectations
          EXPECT_LE(5 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(11 , (_modificationRequest.response.mod_paths.at(0).points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
    

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "path_modification_testFunctionality_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;

}

