#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "evaluation_request_handler.h"
#include "trajectory_request_handler.h"
#include "nav_msgs/Odometry.h"
#include "modifier.h"
#include "population.h"
#include "control_handler.h"
#include "parameter_handler.h"
#include "bezier_curve.h"
#include <type_traits>

struct ModificationResult 
{
  Population popNew_;
  Population transNew_;
  std::vector<uint16_t> i_modified_;
};


enum MotionType 
{
  MT_NONE                         = 0,
  MT_TRANSLATION                  = 1,
  MT_ROTATION                     = 2,
  MT_TRANSLATON_AND_ROTATION      = 3,
};


class Planner {
  public:
    Planner();
    ~Planner();


    /*******************************************
     ************** Data Members ***************
     *******************************************/
    

    // Hold the population of trajectories, 
    // the velocities of each trajectory's segments,
    // the trajectory's path,
    // and the resolution rate for the trajectories
    Population                          population_;
    const float                         resolutionRate_;
    
    // Hold the start and goal configurations
    // and the ranges for each DOF
    MotionState start_;
    MotionState goal_;
    std::vector<Range> ranges_;

    // Starting motion state for planning cycle
    MotionState startPlanning_;

    MotionState latestUpdate_;
    
    
    // The most fit trajectory in the population


    // Timer for sending the best trajec
    // Control cycle - used for determining when to update P(t)
    ros::Timer    controlCycleTimer_;
    ros::Duration controlCycle_;
    
    // Timer for doing a modification
    // Planning cycle - used for determining when to update P(t)
    ros::Timer    planningCycleTimer_;
    ros::Duration planningCycle_;
    
    // Sensing cycle
    ros::Duration sensingCycle_;

    // Cycle to check imminent collision
    ros::Timer imminentCollisionTimer_;
    ros::Duration imminentCollisionCycle_;
   
    
    // Robot ID
    int id_;

    std::vector<double> COLL_DISTS;
    int i_COLL_DISTS_;

    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/


    
    // Start planning
    trajectory_msgs::JointTrajectoryPoint prepareForTestCase();
    void planningCycles(int num);
    void go();
    void goTest(float sec=-1);
    
    // Initialization 
    void initPopulation();
    void init(const uint8_t             i,                
              const ros::NodeHandle&    h, 
              const MotionState         s,                
              const MotionState         g, 
              const std::vector<Range>  r,                
              const int                 population_size, 
              const bool                sub_populations,  
              const TrajectoryType      pop_type=HYBRID,
              const int                 gens_before_cc=0,
              const double              t_pc_rate=2.,
              const double              t_fixed_cc=2.,
              const bool                errorReduction=0);
    
    // Send the best trajectory to the control package
    void sendBest();
    
    // Send the whole population to the trajectory viewer
    void sendPopulation(const Population& pop) const;
    void displayTrajectory(const ramp_msgs::RampTrajectory traj) const;

    // Evaluate the population 
    void evaluateTrajectory(RampTrajectory& t, bool full=true) const;
    void evaluatePopulation();
    
    // Modify trajectory or path
    const std::vector<Path> modifyPath();

    void modifyTrajec(std::vector<RampTrajectory>& result);




    // Build a srv for 1 trajectory with 1-2 curves
    void buildTrajectorySrv(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, ramp_msgs::TrajectorySrv& result, const int id=-1) const;

    // Build a srv for 1 trajectory with no curves
    void buildTrajectorySrv(const Path path, ramp_msgs::TrajectorySrv& result, const int id=-1) const;


    // Build a request for 1 trajectory with 1-2 curves
    void buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, ramp_msgs::TrajectoryRequest& result, const int id=-1) const;

    // Build a request for 1 trajectory with 0 curves
    void buildTrajectoryRequest(const Path path, ramp_msgs::TrajectoryRequest& result, const int id=-1) const;

    





    void buildEvaluationSrv(std::vector<RampTrajectory>& trajecs, ramp_msgs::EvaluationSrv& result) const;
    void buildEvaluationSrv(const RampTrajectory& trajec, ramp_msgs::EvaluationSrv& result) const;
    void buildEvaluationRequest(const RampTrajectory& trajec, ramp_msgs::EvaluationRequest& result, bool full=true) const;


    // Request information from other packages

    // Many trajectories
    void requestTrajectory(ramp_msgs::TrajectorySrv& tr, std::vector<RampTrajectory>& result, const int id=-1);

    // Many trajectories
    void requestTrajectory(std::vector<ramp_msgs::TrajectoryRequest>& trs, std::vector<RampTrajectory>& result);

    // One trajectory 
    void requestTrajectory(ramp_msgs::TrajectoryRequest& tr, RampTrajectory& result);
    void requestTrajectory(const Path p, RampTrajectory& result, const int id=-1);


    // Many trajectories
    void requestEvaluation(std::vector<RampTrajectory>& trajecs);

    // One trajectory
    void requestEvaluation(ramp_msgs::EvaluationRequest& request) const;
    void requestEvaluation(RampTrajectory& t, bool full=true) const;



    // Updates the paths in P(t) so we can get new trajectories
    const uint8_t getNumThrowawayPoints(const RampTrajectory traj, const ros::Duration dur) const;
    
    const double updateCurvePos(const RampTrajectory& traj, const ros::Duration& d) const;


    const ramp_msgs::BezierCurve               handleCurveEnd(const RampTrajectory traj) const;
    

    void adaptCurves     (const MotionState& ms, const ros::Duration& d, std::vector<ramp_msgs::BezierCurve>& result);
    void adaptPaths      (const MotionState& ms, const ros::Duration& d, std::vector<Path>& result);
    void adaptPopulation (const MotionState& ms, const ros::Duration& d);

    // Display all of the paths
    const std::string pathsToString() const;

    // Set the transformation from odometry to world CS
    void setT_base_w(std::vector<double> base_pos);

    // Sets the m_i vector
    const std::vector<MotionState> setMi(const RampTrajectory& trj_current) const;

    std::vector<RampTrajectory> ob_trajectory_;


    const MotionType findMotionType(const ramp_msgs::Obstacle ob) const;
    const ramp_msgs::RampTrajectory getPredictedTrajectory(const ramp_msgs::Obstacle ob) const;
    const ramp_msgs::Path getObstaclePath(const ramp_msgs::Obstacle ob, const MotionType mt) const;
    
    void sensingCycleCallback     (const ramp_msgs::ObstacleList& msg);
    void updateCallback(const ramp_msgs::MotionState& msg);

    /** Data */

    // Motion state that should be reached by next control cycle
    MotionState m_cc_;
    



    // Hold the difference between previous startPlanning 
    // and latestUpdate for each control cycle
    std::vector<MotionState> SP_LU_diffs_;
    const MotionState findAverageDiff();
  

    // Debugging variables
    bool modifications_;
    bool evaluations_;
    bool seedPopulation_;
    
  //private:
    /** These are (mostly) utility members that are only used by Planner and should not be used by other classes */


    /***** Methods *****/

    const Path getRandomPath  (const MotionState s, const MotionState g) const;
    const Path getAdjustedPath(const MotionState s, const MotionState g) const;
    
    // Initialize start and goal
    void initStartGoal(const MotionState s, const MotionState g);
    

    // Returns a unique id for a RampTrajectory 
    const unsigned int getIRT();

    // Modification procedure
    void modification();

    // Callback methods for ros::Timers
    void controlCycleCallback     (const ros::TimerEvent& t);
    void planningCycleCallback    ();
    void imminentCollisionCallback(const ros::TimerEvent& t);

    void resetStart();
    
    bool EC, mod_worked, modded_two;
    
    
   
    // Misc
    const MotionState randomizeMSPositions(const MotionState ms)  const ;
          void randomMS(MotionState& result)                      const ;
          void checkTrajChange()                                        ;
          void seedPopulation()                                         ;
          void seedPopulationTwo()                                      ;

    const RampTrajectory getBestTransTrajectory(const RampTrajectory moving,
                                                const RampTrajectory target);



    const MotionState     errorCorrection();



    

    double  getEarliestStartTime(const RampTrajectory& from);
    void    getTransPop(const Population& pop, const RampTrajectory& movingOn, const double& t_start, Population& result);
    void    switchTrajectory(const RampTrajectory& from, const RampTrajectory& to, const double& t_start, RampTrajectory& result);
    void    computeFullSwitch(const RampTrajectory& from, const RampTrajectory& to, const double& t_start, RampTrajectory& result);


    void getTransPop(const Population& pop, const RampTrajectory& movingOn, Population& result);
    void getTransitionTrajectory(const RampTrajectory& movingOn, const RampTrajectory& trgt_traj, const double& t, RampTrajectory& result);
    void switchTrajectory(const RampTrajectory& from, const RampTrajectory& to, std::vector<RampTrajectory>& result);
    void computeFullSwitch(const RampTrajectory& from, const RampTrajectory& to, RampTrajectory& result);
    
    
    const bool checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to)
      const;
    
   





    const ramp_msgs::BezierCurve replanCurve(const RampTrajectory trajec, const MotionState ms_start) const;
    const RampTrajectory replanTrajec(const RampTrajectory trajec, const MotionState ms_start);
    const std::vector<RampTrajectory> replanTrajecs(const std::vector<RampTrajectory> trajecs, const MotionState ms_start);
    const std::vector<RampTrajectory> getTrajectories(const std::vector<Path>& p);
    const std::vector<RampTrajectory> getTrajectories(std::vector<ramp_msgs::TrajectoryRequest>& tr);
    void updatePathsStart(const MotionState s);




    // 1 if before, 2 if on curve, 3 if past curve 
    const int estimateIfOnCurve(const MotionState ms, 
                                const ramp_msgs::BezierCurve curve) const;

    const std::vector<Path> getRandomPaths        ( const MotionState init, const MotionState goal);
    const std::vector<Path> getAdjustedPaths      ( const MotionState init, const MotionState goal);
    const Population        getPopulation         ( const MotionState init, 
                                                    const MotionState goal, 
                                                    const bool        random = false );


    // Work for CC
    void doControlCycle();

    // Returns the index in the trajectory's path to start checking if the robot has passed it
    const uint8_t getIndexStartPathAdapting(const RampTrajectory t) const;


    // Returns true if motion state satisfies constraints to be a knot point in Path p 
    const bool validKPForPath(const MotionState ms, const Path p) const;

    // Methods for debugging
    void stopForDebugging();
    void restartAfterDebugging();
    void pause();




    void offsetTrajectory(RampTrajectory&  t   ,   const MotionState& diff) ;
    void offsetPopulation(const MotionState& diff) ;
    
    MotionState diff_;
    
    bool predictTransition(const RampTrajectory& from, const RampTrajectory& to, const double& t);


    void reportData() ;


    ros::Duration d_IC_;
    ros::Time t_IC_;

    ros::Time t_prev_update_;
    ros::Time t_prevIC_;
    ros::Time t_prevObIC_;
    ros::Timer ob_dists_timer_;
    double ob_dists_timer_dur_;
    void obICCallback(const ros::TimerEvent& e);
    std::vector<double> ob_dists_;
    std::vector<double> ob_dists_from_obs_;


    /***** Data members *****/

    // Utility instance
    Utility             utility_; 

    // Size of population
    unsigned int        populationSize_;

    // Generation counter
    unsigned int        generation_;

    // ID counter for trajectories
    unsigned int        i_rt;

    // Last time P(t) was updated
    ros::Time           lastUpdate_;

    // How far we need to get to the goal before stopping
    float               goalThreshold_;

    // Number of modification operators 
    unsigned int        num_ops_;

    // Distance threshold for imminent collision
    float               D_;

    // Index of previous best trajectory
    unsigned int        i_best_prev_;

    // Number of generations to wait before starting control cycles
    unsigned int        generationsBeforeCC_;

    // Maximum number of generations to occur between control cycles
    unsigned int        generationsPerCC_;

    // Flag for if the control cycles have started
    bool                cc_started_;

    // Flag for if sub-populations are being used
    bool                subPopulations_;
    
    // Number of planning cycles since last control cycle
    int                 c_pc_;

    // Threshold for getting transition trajectory
    double              transThreshold_;

    // Number of Control Cycles that have occurred
    int                 num_cc_;

    // Distance between each knot points
    double              L_;

    // Type of trajectories the population will hold
    TrajectoryType      pop_type_;

    // Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    EvaluationRequestHandler*   h_eval_req_;
    ControlHandler*             h_control_;
    Modifier*                   modifier_;

    // Parameter handler
    ParameterHandler            h_parameters_;


    double delta_t_switch_;

    double          t_fixed_cc_;
    RampTrajectory  movingOnCC_;
    RampTrajectory  movingOn_;

    // Error Reduction variables
    bool                      errorReduction_;
    std::vector<MotionState>  m_i_;
    
    // Stop things for debugging
    bool stop_;


    bool imminent_collision_;


    bool moving_on_coll_;


    ros::Time t_prevCC_;
    uint8_t pc_switch_;


    ros::Time t_start_;


    tf::Transform T_w_odom_;

    std::vector<double> error_correct_val_pos_, error_correct_val_or_;
    double avg_error_correct_val_pos_, avg_error_correct_val_or_;
    

    std::vector<ros::Duration> adapt_durs_;
    std::vector<ros::Duration> trans_durs_;
    std::vector<ros::Duration> cc_durs_;
    std::vector<ros::Duration> mutate_durs_;
    std::vector<ros::Duration> pc_durs_;
    std::vector<ros::Duration> sc_durs_;
    std::vector<ros::Duration> trajec_durs_;
    std::vector<ros::Duration> eval_durs_;
    std::vector<ros::Duration> error_correct_durs_;
    double avg_adapt_dur_, avg_trans_dur_, avg_cc_dur_, avg_mutate_dur_, avg_pc_dur_, avg_trajec_dur_, 
           avg_eval_dur_, avg_error_correct_dur_, avg_sc_dur_;


    bool reset_;
    bool log_enter_exit_;
    bool log_switching_;
    int num_mods_;
    int num_succ_mods_;
};

#endif
