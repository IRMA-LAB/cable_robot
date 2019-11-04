#include "state_estimation/state_estimator_base.h"

void StateEstimatorBase::EstimatePlatformPose(
  const vect<ActuatorStatus>& active_actuators_status, grabcdpr::RobotVars& robot_vars)
{
  // Extract cable's variables from current actuators status
  std::vector<double> cables_length(robot_vars.cables.size(), 0);
  std::vector<double> swivel_angles(robot_vars.cables.size(), 0);
  for (uint i = 0; i < robot_vars.cables.size(); ++i)
  {
    cables_length[i] = active_actuators_status[i].cable_length;
    swivel_angles[i] = active_actuators_status[i].pulley_angle;
  }
  // Take platform pose from latest known/computed value, possibly all zeros
  grabnum::VectorXd<POSE_DIM> init_guess_pose = robot_vars.platform.pose;

  // Solve direct kinematics
  static const bool kUseGsJacobian = true;
  static const uint8_t kNMax = 10;
  grabnum::VectorXd<POSE_DIM> new_pose =
    SolveDK0(cables_length, swivel_angles, init_guess_pose, params_, kUseGsJacobian,
             kNMax);
  // Update inverse kinematics
  grabcdpr::UpdateIK0(new_pose.GetBlock<3, 1>(1, 1), new_pose.GetBlock<3, 1>(4, 1),
                      params_, robot_vars);
}
