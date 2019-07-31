#ifndef CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
#define CABLE_ROBOT_STATE_ESTIMATOR_BASE_H

#include "cdpr_types.h"
#include "kinematics.h"

class StateEstimatorBase
{
 public:
  StateEstimatorBase() {}
  virtual ~StateEstimatorBase() {}

  virtual void EstimatePlatformPose(grabcdpr::RobotVars& robot_vars);
};

#endif // CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
