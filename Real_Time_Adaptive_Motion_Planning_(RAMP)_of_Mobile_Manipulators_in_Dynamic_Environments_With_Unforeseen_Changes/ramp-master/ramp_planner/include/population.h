#ifndef POPULATION_H
#define POPULATION_H

#include "ramp_trajectory.h"
#include "utility.h"
#include "ramp_msgs/Population.h"


class Population {
  public:

    Population();
    Population(const unsigned int size, const TrajectoryType type, const bool isSubPop=false);

    /* Methods */
    const unsigned int    size() const;
    const int             add(const RampTrajectory& rt);
    const int             calcBestIndex() const;
    void                  clear();
    void                  replace(const uint8_t i, const RampTrajectory& trajec);
    void                  replaceAll(const std::vector<RampTrajectory>& new_pop);
    const RampTrajectory  get(const unsigned int i) const;
    const int             getIndexFromId(const uint16_t id) const;
    const std::vector<RampTrajectory> getTrajectories() const;
    const double          getMinFitness() const;
    const bool            contains(const RampTrajectory& rt) const;
    const bool            feasibleExists() const;
    const bool            infeasibleExists() const;
    const bool            replacementPossible(const RampTrajectory& rt) const;
    const bool            canReplace(const RampTrajectory& rt, const int& i) const;
    const int             getReplacementID(const RampTrajectory& rt) const;
    const int             getNumSubPops() const;
    const RampTrajectory  getBest() const;

    const ros::Duration   getEarliestStartTime() const;
    void setStartTime(double t_start);
   
    const std::vector<RampTrajectory> getBestFromSubPops() const;
    const std::vector<Population> createSubPopulations(const double delta_theta=PI/18.f);


    const std::string     fitnessFeasibleToString() const;
    const std::string     toString() const;
    ramp_msgs::Population populationMsg() const;

    /** Data Members */
    std::vector<Path>           paths_;
    TrajectoryType              type_; 
    unsigned int                maxSize_;

    double                      t_start_;
    
    std::vector<RampTrajectory> trajectories_;
    
  private:
    std::vector<Population>     subPopulations_;
    bool                        isSubPopulation_;
    Utility                     utility_;
};

#endif
