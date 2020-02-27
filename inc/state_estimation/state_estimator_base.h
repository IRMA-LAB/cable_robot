/**
 * @file controller_joints_pvt.h
 * @author Simone Comari
 * @date 27 Feb 2020
 * @brief This file includes a base class implementation of CDPR state estimator.
 */

#ifndef CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
#define CABLE_ROBOT_STATE_ESTIMATOR_BASE_H

#include "cdpr_types.h"
#include "kinematics.h"

#include "utils/types.h"

class StateEstimatorBase
{
 public:
  StateEstimatorBase(const grabcdpr::RobotParams& params) : params_(params) {}
  virtual ~StateEstimatorBase() {}

  virtual bool estimatePlatformPose(const vect<ActuatorStatus>& actuators_status,
                                    grabcdpr::RobotVars& robot_vars);

  virtual void reset();

 protected:
  grabcdpr::RobotParams params_;
  vect<ActuatorStatus> prev_actuators_status_;
};

#endif // CABLE_ROBOT_STATE_ESTIMATOR_BASE_H
