/* 
 * File:   trajectory_evaluation_testPerformance.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 9, 2015, 3:42 PM
 */


// include header file of the fixture tests
#include "trajectory_evaluation_fixtureTest.h"


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_50ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.050).toSec(), _duration)
                    <<"The trajectory generator spun more than 50 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_45ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.045).toSec(), _duration)
                    <<"The trajectory generator spun more than 45 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_40ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.040).toSec(), _duration)
                    <<"The trajectory generator spun more than 40 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_35ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.035).toSec(), _duration)
                    <<"The trajectory generator spun more than 35 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_30ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.030).toSec(), _duration)
                    <<"The trajectory generator spun more than 30 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_25ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.025).toSec(), _duration)
                    <<"The trajectory generator spun more than 25 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_20ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.020).toSec(), _duration)
                    <<"The trajectory generator spun more than 20 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_15ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.015).toSec(), _duration)
                    <<"The trajectory generator spun more than 15 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_10ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.010).toSec(), _duration)
                    <<"The trajectory generator spun more than 10 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points_In_05ms){
    
    ramp_msgs::RampTrajectory _trajectory;
    trajectory_msgs::JointTrajectory _jointTrajectory;
    
    // Seed motion states and push them into trajectory    
    { // Scop the variables & Seed the first joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.5f);
    _jointTrajectoryPoint.positions.push_back(2.f);
    _jointTrajectoryPoint.positions.push_back((-3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(-0.23f);
    _jointTrajectoryPoint.velocities.push_back(0.22f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------
    
    { // Scop the variables & Seed the second joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(1.75f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((-1.f*PI/2.f));
    _jointTrajectoryPoint.velocities.push_back(-0.20f);
    _jointTrajectoryPoint.velocities.push_back(-0.15f);
    _jointTrajectoryPoint.velocities.push_back(-0.13f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    { // Scop the variables & Seed the third joint point 
    trajectory_msgs::JointTrajectoryPoint _jointTrajectoryPoint;
    _jointTrajectoryPoint.positions.push_back(3.25f);
    _jointTrajectoryPoint.positions.push_back(0.f);
    _jointTrajectoryPoint.positions.push_back((3.f*PI/4.f));
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectoryPoint.velocities.push_back(0.f);
    _jointTrajectory.points.push_back(_jointTrajectoryPoint);
    }
    
    // -----------------------------------------------------

    // Initialize the trajectory's arguments for evaluation request 
    _trajectory.trajectory = _jointTrajectory;
    _trajectory.id = 1;
    _trajectory.feasible = true;
    _trajectory.fitness = -1;  
    _trajectory.t_firstCollision = ros::Duration(9999.f);
    _trajectory.t_start          = ros::Duration(2.0f);
    // -----------------------------------------------------

    ramp_msgs::EvaluationRequest er;
    er.trajectory = _trajectory;
    er.currentTheta = (PI/4.f);

    // Initialize the evaluation request -------------------
    _evaluationSrv.request.reqs.push_back(er);
    //_evaluationSrv.request.trajectory = _trajectory;
    //_evaluationSrv.request.currentTheta = (PI/4.f);
    // -----------------------------------------------------

    try{          
          // Save the start time.
          double _startTime = ros::Time::now().toSec();
          
          // Request a trajectory evaluation
          _client.call(_evaluationSrv);
          
          // Save the end time.
          double _endTime = ros::Time::now().toSec();

          // Duration of a trajectory request evaluation
          double _duration = (_endTime - _startTime);
          // Expected trajectory request doesn't exceed 50 milliseconds
          EXPECT_GE(ros::Duration(0.005).toSec(), _duration)
                    <<"The trajectory generator spun more than 5 milliseconds.";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


//============= Main function of test runner ==================================
int main(int argc, char **argv) {
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_evaluation_testPerformance_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
    
}
//=============================================================================



