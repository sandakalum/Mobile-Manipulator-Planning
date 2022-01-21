#ifndef PREDICTION_H
#define PREDICTION_H
#include "line.h"
#include "circle.h"
#include "utility.h"




class Prediction {
public:
  Prediction();
  ~Prediction();

  // Service callback, the input is a path and the output a trajectory
  bool trajectoryRequest(ramp_msgs::TrajectoryRequest& req, ramp_msgs::TrajectoryResponse& res);
private:

  ramp_msgs::Path path_;

  void init(const ramp_msgs::TrajectoryRequest req);

  Utility utility_;
};

#endif
