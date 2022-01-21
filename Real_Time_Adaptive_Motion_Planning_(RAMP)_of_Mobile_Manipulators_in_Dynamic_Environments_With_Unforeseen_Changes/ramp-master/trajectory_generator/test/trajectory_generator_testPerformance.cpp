/* 
 * File:   trajectory_generator_testPerformance.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 10:13 AM
 */


// include header file of the fixture tests
#include "trajectory_generator_fixtureTest.h"

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_50ms){

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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.050).toSec(), _duration)
                    <<"The trajectory generator spun more than 50 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_45ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 45 milliseconds
          EXPECT_GE(ros::Duration(0.045).toSec(), _duration)
                    <<"The trajectory generator spun more than 45 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_40ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 40 milliseconds
          EXPECT_GE(ros::Duration(0.040).toSec(), _duration)
                    <<"The trajectory generator spun more than 40 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_35ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't 35 milliseconds
          EXPECT_GE(ros::Duration(0.035).toSec(), _duration)
                    <<"The trajectory generator spun more than 35 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_30ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 30 milliseconds
          EXPECT_GE(ros::Duration(0.030).toSec(), _duration)
                    <<"The trajectory generator spun more than 30 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_25ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 25 milliseconds
          EXPECT_GE(ros::Duration(0.025).toSec(), _duration)
                    <<"The trajectory generator spun more than 25 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_20ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 20 milliseconds
          EXPECT_GE(ros::Duration(0.020).toSec(), _duration)
                    <<"The trajectory generator spun more than 20 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_15ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 15 milliseconds
          EXPECT_GE(ros::Duration(0.015).toSec(), _duration)
                    <<"The trajectory generator spun more than 15 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_10ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 10 milliseconds
          EXPECT_GE(ros::Duration(0.010).toSec(), _duration)
                    <<"The trajectory generator spun more than 10 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}

TEST_F(trajectoryGeneratorFixtureTest, testTrajectoryRequest_Path_3KnotPoints_05ms){
    
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
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory
          _client.call(_trajectorySrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 5 milliseconds
          EXPECT_GE(ros::Duration(0.005).toSec(), _duration)
                    <<"The trajectory generator spun more than 5 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory generator service.";
    }
}


//============= Main function of test runer ===================================
int main(int argc, char **argv) {
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_generator_testPerformance_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
    
}

//=============================================================================
