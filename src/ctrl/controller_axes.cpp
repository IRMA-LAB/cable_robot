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
  grabcdpr::UpdateIK0(position, orientation, params_, updated_vars);

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
      const grabnum::VectorXi<POSE_DIM> kMask({1, 1, 1, 0, 0, 0});
      position            = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec3 solution = nonLinsolveJacGeomStatic(target_pose_, kMask);
      orientation         = arma::conv_to<vectD>::from(solution);
      break;
    }
    case 4:
    {
      // Calculate feasible orientation for given position
      const grabnum::VectorXi<POSE_DIM> kMask({1, 1, 1, 1, 0, 0});
      position            = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec2 solution = nonLinsolveJacGeomStatic(target_pose_, kMask);
      orientation(4)      = target_pose_(4); // index starts at 1
      orientation(5)      = solution(4); // index starts at 0
      orientation(6)      = solution(5); // index starts at 0
      break;
    }
    case 5:
    {
      // Calculate feasible orientation for given position
      const grabnum::VectorXi<POSE_DIM> kMask({1, 1, 1, 1, 1, 0});
      position           = target_pose_.GetBlock<3, 1>(1, 1);
      arma::vec solution = nonLinsolveJacGeomStatic(target_pose_, kMask); // "scalar"
      orientation(4)     = target_pose_(4); // index starts at 1
      orientation(5)     = target_pose_(5); // index starts at 1
      orientation(6)     = solution(5); // index starts at 0
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

arma::vec
ControllerAxes::nonLinsolveJacGeomStatic(const grabnum::VectorXd<POSE_DIM>& init_guess,
                                         const grabnum::VectorXi<POSE_DIM>& mask,
                                         const uint8_t nmax /*= 100*/) const
{
  static const double kFtol = 1e-4;
  static const double kXtol = 1e-3;

  // Distribute initial guess between fixed and variable coordinates (i.e. the solution of
  // the iterative process)
  arma::vec fixed_coord(mask.NonZeros().size(), arma::fill::none);
  arma::vec var_coord(POSE_DIM - fixed_coord.n_elem, arma::fill::none);
  ulong fixed_idx = 0;
  ulong var_idx   = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i) == 1)
      fixed_coord(fixed_idx++) = init_guess(i);
    else
      var_coord(var_idx++) = init_guess(i);

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  grabcdpr::calcGeometricStatic(params_, fixed_coord, var_coord, mask, func_jacob,
                                func_val);

  // Init iteration variables
  arma::vec s;
  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  // Start iterative process
  while (iter < nmax && arma::norm(func_val) > kFtol && err > cond)
  {
    iter++;
    s = arma::solve(func_jacob, func_val);
    var_coord -= s;
    grabcdpr::calcGeometricStatic(params_, fixed_coord, var_coord, mask, func_jacob,
                                  func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(var_coord));
  }

  return var_coord;
}

bool ControllerAxes::isPoseReachable(grabcdpr::RobotVars& vars) const
{
  // Calculate cables tensions
  grabcdpr::UpdateExternalLoads(Matrix3d(1.0), params_.platform, vars.platform);
  grabcdpr::CalCablesTensionStat(vars);
  // Check tension is within feasible range for all cables
  return (vars.tension_vector.min() >= kMinCableTension_ &&
          vars.tension_vector.max() <= kMaxCableTension_);
}
