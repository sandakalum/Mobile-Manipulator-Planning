
// include header file of the fixture tests
#include "fullPath_testFunctionality.h"

TEST_F(fullPathFixtureTest, testEvaluationRequest_JointTrajectory_With_7_Point){

//    // Initialize the evaluation request -------------------
//    _evaluationRequest.trajectory = trajectoryBulider(7);
//    _evaluationRequest.currentTheta = (PI/4.f);
//
//    //Push the evaluation request back into the evaluation request server
//    _evaluationServer.request.reqs.push_back(_evaluationRequest);
//    // -----------------------------------------------------
//    
//    try{          
//          // Request trajectory evaluation
//          _client.call(_evaluationServer);
//          
//          // Expectations  
//          EXPECT_LE(0, (_evaluationServer.response.resps.at(0).fitness))
//                    <<"The evaluation execution is successfully done!";
//          EXPECT_GE(1, (_evaluationServer.response.resps.at(0).fitness))
//                    <<"The evaluation execution is successfully done!";
//
//    }catch(...){
//        FAIL() << "Failed to call trajectory evaluation component.";
//    }
}

//============= Main function of test runner ==================================
int main(int argc, char **argv) {
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "fullPath_testFunctionality_runner");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
    
}
//=============================================================================


    