/**
 * @file controller_joints_pvt.cpp
 * @author Simone Comari
 * @date 27 Feb 2020
 * @brief This file includes definitions of class present in controller_joints_pvt.h.
 */

#include "state_estimation/state_estimator_base.h"

bool StateEstimatorBase::estimatePlatformPose(
  const vect<ActuatorStatus>& actuators_status, grabcdpr::RobotVars& robot_vars)
{
  if (prev_actuators_status_.empty())
    // No previous reference (first call after reset/instanciation) --> copy values
    // straight from actuators status
    for (uint i = 0; i < robot_vars.cables.size(); ++i)
    {
      robot_vars.cables[i].length     = actuators_status[i].cable_length;
      robot_vars.cables[i].swivel_ang = actuators_status[i].pulley_angle;
    }
  else
    // Extract cable's variables changes from current actuators status to update
    // corresponding robot cable's variable
    for (uint i = 0; i < robot_vars.cables.size(); ++i)
    {
      robot_vars.cables[i].length +=
        (actuators_status[i].cable_length - prev_actuators_status_[i].cable_length);
      robot_vars.cables[i].swivel_ang +=
        (actuators_status[i].pulley_angle - prev_actuators_status_[i].pulley_angle);
    }
  // Update previous status for next call
  prev_actuators_status_ = actuators_status;

  return grabcdpr::updateDK0(params_, robot_vars);
}

void StateEstimatorBase::reset() { prev_actuators_status_.clear(); }
