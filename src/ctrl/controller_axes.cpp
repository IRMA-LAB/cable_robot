#include "ctrl/controller_axes.h"

ControllerAxes::ControllerAxes(const vect<id_t>& motors_id, const uint32_t period_nsec,
                               const grabcdpr::RobotParams& params)
  : ControllerBase(motors_id), params_(params), change_target_(false), target_set_(false)
{
  SetMode(ControlMode::CABLE_LENGTH);
  delta_move_.SetZero();
  abs_delta_move_per_cycle_ = period_nsec * 0.000000001 * kAbsDeltaMovePerSec_;
}

//--------- Public functions ---------------------------------------------------------//

void ControllerAxes::setTargetPosition(const grabnum::Vector3d& target_pos)
{
  change_target_   = false;
  target_set_      = true;
  target_position_ = target_pos;
}

void ControllerAxes::singleAxisIncrement(const Axis axis, const bool active,
                                         const Sign sign /*= Sign::POS*/)
{
  if (active == change_target_)
    return;

  change_target_ = active;
  target_set_    = active;
  if (change_target_)
  {
    delta_move_.SetZero();
    delta_move_(axis) = sign * abs_delta_move_per_cycle_;
  }
}

void ControllerAxes::stop()
{
  change_target_ = false;
  target_set_    = false;
}

vect<ControlAction> ControllerAxes::calcCtrlActions(const grabcdpr::RobotVars& vars,
                                                    const vect<ActuatorStatus>&)
{
  vect<ControlAction> ctrl_actions(motors_id_.size());
  if (!target_set_)
    return ctrl_actions;

  // Update target pose (define new target position and use current orientation)
  if (change_target_)
    target_pose_ = vars.platform.pose + delta_move_;
  else
  {
    // Check if position was reached (within 1mm tolerance)
    if (vars.platform.position.IsApprox(target_position_, 0.001))
    {
      target_set_ = false;
      return ctrl_actions;
    }
    // Because we work in quasi-static conditions, cannot assign a too large target change
    interpPosTarget(vars.platform.position); // set position in target_pose_
    target_pose_.SetBlock<3, 1>(4, 1, vars.platform.orientation);
  }

  // Find feasible target pose according to number of cables (under/fully/over-actuated)
  grabnum::Vector3d position, orientation;
  if (!calcRealTargetPose(position, orientation))
    return ctrl_actions;

  // Calculate cables lenght targets from inverse kinematics
  grabcdpr::RobotVars updated_vars;
  grabcdpr::updateIK0(position, orientation, params_, updated_vars);

  if (isPoseReachable(updated_vars))
    // Assign new targets to drives
    for (uint i = 0; i < ctrl_actions.size(); i++)
      ctrl_actions[i].cable_length = updated_vars.cables[i].length;

  return ctrl_actions;
}

//--------- Private functions --------------------------------------------------------//

void ControllerAxes::interpPosTarget(const grabnum::Vector3d& current_pos)
{
  double distance = grabnum::Norm(current_pos - target_position_);
  if (distance <= abs_delta_move_per_cycle_)
  {
    target_pose_.SetBlock<3, 1>(1, 1, target_position_);
    return;
  }
  // Else interpolate (from parametric representation of a 3D line passing by two points)
  double t        = abs_delta_move_per_cycle_ / distance; // (0:1)
  target_pose_(X) = current_pos(X) + (target_position_(X) - current_pos(X)) * t;
  target_pose_(Y) = current_pos(Y) + (target_position_(Y) - current_pos(Y)) * t;
  target_pose_(Z) = current_pos(Z) + (target_position_(Z) - current_pos(Z)) * t;
}

bool ControllerAxes::calcRealTargetPose(grabnum::Vector3d& position,
                                        grabnum::Vector3d& orientation) const
{
  switch (motors_id_.size())
  {
    case 3:
    {
      // Calculate feasible orientation for given position
      const arma::uvec6 kMask({1, 1, 1, 0, 0, 0});
      position = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec3 solution =
        grabcdpr::nonLinsolveJacGeomStatic(target_pose_, kMask, params_);
      orientation = arma::conv_to<vectD>::from(solution);
      break;
    }
    case 4:
    {
      // Calculate feasible orientation for given position
      const arma::uvec6 kMask({1, 1, 1, 1, 0, 0});
      position = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec2 solution =
        grabcdpr::nonLinsolveJacGeomStatic(target_pose_, kMask, params_);
      orientation(4) = target_pose_(4); // index starts at 1
      orientation(5) = solution(0);     // index starts at 0
      orientation(6) = solution(1);     // index starts at 0
      break;
    }
    case 5:
    {
      // Calculate feasible orientation for given position
      const arma::uvec6 kMask({1, 1, 1, 1, 1, 0});
      position = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec solution =
        grabcdpr::nonLinsolveJacGeomStatic(target_pose_, kMask, params_); // "scalar"
      orientation(4) = target_pose_(4); // index starts at 1
      orientation(5) = target_pose_(5); // index starts at 1
      orientation(6) = solution(0);     // index starts at 0
      break;
    }
    case 6:
      // Take directly user input
      position    = target_pose_.GetBlock<3, 1>(1, 1);
      orientation = target_pose_.GetBlock<3, 1>(4, 1);
      break;
    case 7:
    case 8:
      // TODO: over-actuated case
      //      break;
    default:
      return false;
  }
  return true;
}

bool ControllerAxes::isPoseReachable(grabcdpr::RobotVars& vars) const
{
  // Calculate cables tensions
  grabcdpr::updateExternalLoads(params_.platform, vars.platform);
  grabcdpr::updateCablesStaticTension(vars);
  // Check tension is within feasible range for all cables
  return (vars.tension_vector.min() >= kMinCableTension_ &&
          vars.tension_vector.max() <= kMaxCableTension_);
}
