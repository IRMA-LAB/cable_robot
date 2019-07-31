#include "ctrl/controller_axes.h"

ControllerAxes::ControllerAxes(const vect<id_t>& motors_id, const uint32_t period_nsec,
                               const grabcdpr::RobotParams& params)
  : ControllerBase(motors_id), params_(params), period_sec_(period_nsec * 0.000000001),
    change_target_(false), target_set_(false)
{
  SetMode(ControlMode::CABLE_LENGTH);
  delta_move_.SetZero();
}

//--------- Public functions ---------------------------------------------------------//

void ControllerAxes::setAxesTarget(const grabnum::Vector3d& target_pos)
{
  change_target_ = false;
  target_set_    = true;
  target_pose_.SetBlock<3, 1>(1, 1, target_pos);
}

void ControllerAxes::SingleAxisIncrement(const Axis axis, const bool active,
                                         const Sign sign /*= Sign::POS*/)
{
  if (active == change_target_)
    return;

  change_target_ = active;
  target_set_    = active;
  if (change_target_)
  {
    delta_move_.SetZero();
    delta_move_(axis) = sign * kAbsDeltaMovePerSec_ * period_sec_;
  }
}

void ControllerAxes::stop()
{
  change_target_ = false;
  target_set_    = false;
}

vect<ControlAction> ControllerAxes::CalcCtrlActions(const grabcdpr::RobotVars& vars,
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
    target_pose_.SetBlock<3, 1>(4,1, vars.platform.orientation);
    // Check if position was reached
    if (vars.platform.position.IsApprox(target_pose_.GetBlock<3, 1>(1,1)))
    {
      target_set_ = false;
      return ctrl_actions;
    }
  }
  // Calculate feasible orientation for given position
  grabnum::Vector3d orientation = NonLinsolveJacGeomStatic(target_pose_);
  // Calculate cables lenght targets from inverse kinematics
  grabcdpr::RobotVars updated_vars;
  grabcdpr::UpdateIK0(target_pose_.GetBlock<3, 1>(1, 1), orientation, params_,
                      updated_vars);
  if (isPoseReachable(updated_vars))
    // Assign new targets to drives
    for (uint i = 0; i < ctrl_actions.size(); i++)
      ctrl_actions[i].cable_length = updated_vars.cables[i].length;

  return ctrl_actions;
}

//--------- Private functions --------------------------------------------------------//

grabnum::Vector3d
ControllerAxes::NonLinsolveJacGeomStatic(const grabnum::VectorXd<POSE_DIM>& init_guess,
                                         const uint8_t nmax /*= 100*/) const
{
  // TODO: check how to properly implement this
  static const grabnum::VectorXi<POSE_DIM> kMask({1, 1, 1, 0, 0, 0});
  static const double ftol = 1e-9;
  static const double xtol = 1e-7;

  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  cv::Mat F, s, J, solution;

  grabcdpr::calcGeometricStatic(params_, toCvMat(init_guess.GetBlock<3, 1>(1, 1)),
                                toCvMat(init_guess.GetBlock<3, 1>(4, 1)), kMask, J,
                                solution);

  while (iter < nmax && cv::norm(F) > ftol && err > cond)
  {
    iter++;
    cv::solve(J, s, F);
    solution -= s;
    grabcdpr::calcGeometricStatic(params_, toCvMat(init_guess.GetBlock<3, 1>(1, 1)),
                                  toCvMat(init_guess.GetBlock<3, 1>(4, 1)), kMask, J,
                                  solution);
    err  = cv::norm(s);
    cond = xtol * (1 + cv::norm(solution));
  }

  grabnum::Vector3d sol;
  for (uint i = 0; i < 3; i++)
    sol(i + 1) = solution.at<double>(static_cast<int>(i));

  return sol;
}

bool ControllerAxes::isPoseReachable(const grabcdpr::RobotVars& vars) const
{
  // TODO: tension check?
  return true;
}
