#ifndef CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
#define CABLE_ROBOT_STATE_ESTIMATOR_BASE_H

#include "cdpr_types.h"
#include "kinematics.h"

#include "utils/types.h"

class StateEstimatorBase
{
 public:
  StateEstimatorBase(const grabcdpr::RobotParams& params): params_(params) {}
  virtual ~StateEstimatorBase() {}

  virtual void EstimatePlatformPose(const vect<ActuatorStatus>& active_actuators_status,
                                    grabcdpr::RobotVars& robot_vars);

 protected:
  grabcdpr::RobotParams params_;
};

#endif // CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
