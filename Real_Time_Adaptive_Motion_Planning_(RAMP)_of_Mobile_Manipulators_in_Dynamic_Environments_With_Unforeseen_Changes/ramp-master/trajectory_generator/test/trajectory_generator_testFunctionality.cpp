/* 
 * File:   trajectory_generator_testFunctionality.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 9, 2015, 3:42 PM
 */


// include header file of the fixture tests
#include "trajectory_generator_fixtureTest.h"


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_One_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
    }

    ramp_msgs::TrajectoryRequest tr;
    
    // -----------------------------------------------------

    // Initialize the trajectory request -------------------
    tr.path = _path;
    tr.type = PREDICTION;

    _trajectorySrv.request.reqs.push_back(tr);

    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = PREDICTION;
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(100, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Two_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }

    // -----------------------------------------------------
    
    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = PREDICTION;
    
    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = PREDICTION;
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(75, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(150, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Three_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
  
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------
    
    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HOLONOMIC;
    tr.bezierCurves.push_back(_temp);


    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HOLONOMIC;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Four_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);
    
    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Five_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);

    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Six_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);
    
    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Seven_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
    
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);

    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Eight_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);

    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Nine_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

    // Seed knot points and push them into path
    { // Scop the variables & Seed knot point 
    ramp_msgs::KnotPoint knotPoint;
    knotPoint.motionState.positions.push_back(1.5f);
    knotPoint.motionState.positions.push_back(2.f);
    knotPoint.motionState.positions.push_back((-3.f*PI/4.f));
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(-0.23f);
    knotPoint.motionState.velocities.push_back(0.22f);
    knotPoint.motionState.time = 0;    
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------

    
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);
    
    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_With_Ten_KnotPoints){
    
    // A path needed for trajectory request
    ramp_msgs::Path _path;

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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
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
    _path.points.push_back(knotPoint);
    }
    
    // -----------------------------------------------------
  
    // Initialize the bezier curve  ------------------------
     ramp_msgs::BezierCurve _temp;
    _temp.segmentPoints.push_back(_path.points.at(0).motionState);
    _temp.segmentPoints.push_back(_path.points.at(1).motionState);
    _temp.segmentPoints.push_back(_path.points.at(2).motionState);
    // -----------------------------------------------------

    ramp_msgs::TrajectoryRequest tr;
    tr.path = _path;
    tr.type = HYBRID;
    tr.bezierCurves.push_back(_temp);

    // Initialize the trajectory request -------------------
    _trajectorySrv.request.reqs.push_back(tr);
    //_trajectorySrv.request.path = _path;
    //_trajectorySrv.request.type = HYBRID;
    //_trajectorySrv.request.bezierCurves.push_back(_temp);
    // -----------------------------------------------------

    try{          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Expectations
          EXPECT_LE(50, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is less than 50 point";

          EXPECT_GE(500, (_trajectorySrv.response.trajectory.trajectory.points.size()))
                    <<"Size of the trajectory is greater than 75 point";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}



//============= Main function of test runer ===================================
int main(int argc, char **argv) {
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_generator_testFunctionality_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
    
}

//=============================================================================
