/* 
 * File:   trajectory_evaluation_testFunctionality.cpp
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 9, 2015, 3:42 PM
 */


// include header file of the fixture tests
#include "trajectory_evaluation_fixtureTest.h"


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_One_Point){
    
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Two_Points){
    
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Three_Points){
    
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_Four_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_five_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_six_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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

    { // Scop the variables & Seed the sixth joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_seven_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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

    { // Scop the variables & Seed the sixth joint point 
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

    { // Scop the variables & Seed the seventh joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_eight_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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

    { // Scop the variables & Seed the sixth joint point 
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

    { // Scop the variables & Seed the seventh joint point 
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

    { // Scop the variables & Seed the eight joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_nine_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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

    { // Scop the variables & Seed the sixth joint point 
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

    { // Scop the variables & Seed the seventh joint point 
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

    { // Scop the variables & Seed the eight joint point 
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

    { // Scop the variables & Seed the ninth joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


TEST_F(trajectoryEvaluationFixtureTest, testEvaluationRequest_JointTrajectory_With_ten_Points){
    
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
    
    { // Scop the variables & Seed the forth joint point 
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

    { // Scop the variables & Seed the fifth joint point 
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

    { // Scop the variables & Seed the sixth joint point 
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

    { // Scop the variables & Seed the seventh joint point 
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

    { // Scop the variables & Seed the eight joint point 
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

    { // Scop the variables & Seed the ninth joint point 
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

    { // Scop the variables & Seed the tenth joint point 
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
          // Request a trajectory
          _client.call(_evaluationSrv);
          
          // Expectations
          EXPECT_LE((10), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is less than 10";

          EXPECT_GE((30), (_evaluationSrv.response.resps[0].fitness))
                    <<"The evaluation fitness is greater than 30";
          
    }catch(...){
        FAIL() << "Failed to call trajectory evaluation service.";
    }
}


//============= Main function of test runer ===================================
int main(int argc, char **argv) {
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_evaluation_testFunctionality_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
    
}

//=============================================================================


