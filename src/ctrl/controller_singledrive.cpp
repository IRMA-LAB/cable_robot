/**
 * @file controller_singledrive.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 10 Jan 2020
 * @brief File containing definitions of derived class declared in
 * controller_singledrive.h.
 */

#include "ctrl/controller_singledrive.h"

ControllerSingleDrive::ControllerSingleDrive(const uint32_t period_nsec)
  : ControllerBase(), period_sec_(period_nsec * 0.000000001),
    pos_ss_err_tol_(kDefaultPosSsErrTol_), torque_ss_err_tol_(kDefaultTorqueSsErrTol_),
    torque_pid_(period_sec_)
{
  Clear();
  torque_pid_.SetParams(torque_pid_params_);
  abs_delta_torque_ = period_sec_ * kAbsDeltaTorquePerSec_; // delta per cycle
}

ControllerSingleDrive::ControllerSingleDrive(const id_t motor_id,
                                             const uint32_t period_nsec)
  : ControllerBase(vect<id_t>(1, motor_id)), period_sec_(period_nsec * 0.000000001),
    pos_ss_err_tol_(kDefaultPosSsErrTol_), torque_ss_err_tol_(kDefaultTorqueSsErrTol_),
    torque_pid_(period_sec_)
{
  Clear();
  torque_pid_.SetParams(torque_pid_params_);
  abs_delta_torque_ = period_sec_ * kAbsDeltaTorquePerSec_; // delta per cycle
}

//--------- Public functions ---------------------------------------------------------//

void ControllerSingleDrive::SetCableLenTarget(const double target)
{
  Clear();
  length_target_ = target;
  target_flags_.set(LENGTH);
}

void ControllerSingleDrive::SetMotorPosTarget(const int32_t target,
                                              const bool apply_traj /*= true*/,
                                              const double time /*= -1.0*/)
{
  Clear();
  pos_target_true_  = target;
  pos_target_       = static_cast<double>(pos_target_true_);
  traj_time_        = time;
  new_trajectory_   = true;
  apply_trajectory_ = apply_traj;
  target_flags_.set(POSITION);
}

void ControllerSingleDrive::SetMotorSpeedTarget(const int32_t target)
{
  Clear();
  speed_target_true_ = target;
  target_flags_.set(SPEED);
}

void ControllerSingleDrive::SetMotorTorqueTarget(const int16_t target)
{
  Clear();
  torque_target_true_ = target;
  torque_target_      = static_cast<double>(torque_target_true_);
  torque_pid_.Reset();
  target_flags_.set(TORQUE);
}

void ControllerSingleDrive::SetCableLenTrajectory(const std::vector<double>& trajectory)
{
  Clear();
  apply_trajectory_ = true;
  new_trajectory_   = true;
  cable_len_traj_   = trajectory;
  target_flags_.set(LENGTH);
}

void ControllerSingleDrive::CableLenIncrement(const bool active,
                                              const Sign sign /*= Sign::POS*/,
                                              const bool micromove /*= true*/)
{
  if (active == change_length_target_)
    return;

  change_length_target_ = active;
  if (change_length_target_)
  {
    delta_length_ = sign *
                    (micromove ? kAbsDeltaLengthMicroPerSec_ : kAbsDeltaLengthPerSec_) *
                    period_sec_;
  }
}

void ControllerSingleDrive::ScaleMotorSpeed(const double scale)
{
  speed_target_true_ = static_cast<int>(round(scale * kAbsMaxSpeed_));
}

void ControllerSingleDrive::MotorTorqueIncrement(const bool active,
                                                 const Sign sign /*= Sign::POS*/)
{
  if (active == change_torque_target_)
    return;

  change_torque_target_ = active;
  if (change_torque_target_)
    delta_torque_ = sign * abs_delta_torque_;
}

vect<ControlAction>
ControllerSingleDrive::CalcCtrlActions(const grabcdpr::RobotVars &,
                                       const vect<ActuatorStatus>& actuators_status)
{
  ControlAction res;
  if (!modes_.empty())
  {
    res.ctrl_mode = modes_[0];
    res.motor_id  = motors_id_[0];
  }
  switch (res.ctrl_mode)
  {
    case CABLE_LENGTH:
      if (target_flags_.test(LENGTH))
      {
        if (change_length_target_)
          length_target_ += delta_length_;
        if (apply_trajectory_)
          length_target_ = GetTrajectoryPoint();
        res.cable_length = length_target_;
      }
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_POSITION:
      if (target_flags_.test(POSITION))
        res.motor_position = CalcMotorPos(actuators_status);
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_SPEED:
      if (target_flags_.test(SPEED))
        res.motor_speed = speed_target_true_;
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_TORQUE:
      if (target_flags_.test(TORQUE))
      {
        if (change_torque_target_)
        {
          torque_target_ += delta_torque_;
          torque_target_true_ = static_cast<int16_t>(round(torque_target_));
          torque_pid_.Reset();
          on_target_ = false;
        }
        res.motor_torque = CalcMotorTorque(actuators_status);
      }
      else
        res.ctrl_mode = NONE;
      break;
    case NONE:
      res.ctrl_mode = NONE;
      break;
  }
  return vect<ControlAction>(1, res);
}

//--------- Private functions --------------------------------------------------------//

int32_t ControllerSingleDrive::CalcMotorPos(const vect<ActuatorStatus>& actuators_status)
{
  if (on_target_)
    return pos_target_true_;

  int32_t pos_target =
    pos_target_true_; // this is for safety, in case there's no id match
  for (const ActuatorStatus& actuator_status : actuators_status)
  {
    if (actuator_status.id != motors_id_[0])
      continue;
    pos_target =
      CalcPoly5Waypoint(actuator_status.motor_position, pos_target_true_, kAbsMaxSpeed_);
    on_target_ = pos_target == pos_target_true_;
    break;
  }
  return pos_target;
}

int16_t
ControllerSingleDrive::CalcMotorTorque(const vect<ActuatorStatus>& actuators_status)
{
  if (on_target_)
    return torque_target_true_;

  double motor_torque = torque_target_;
  for (const ActuatorStatus& actuator_status : actuators_status)
  {
    if (actuator_status.id != motors_id_[0])
      continue;
    double current_motor_torque = static_cast<double>(actuator_status.motor_torque);
    motor_torque = torque_pid_.Calculate(torque_target_, current_motor_torque);
    //    printf("%d - %.1f -> %.1f\n", torque_target_true_, current_motor_torque,
    //           motor_torque);
    break;
  }
  on_target_ = (std::abs(torque_pid_.GetError()) + std::abs(torque_pid_.GetPrevError())) <
               (2 * torque_ss_err_tol_);
  return static_cast<int16_t>(round(motor_torque));
}

int32_t ControllerSingleDrive::CalcPoly5Waypoint(const int32_t q, const int32_t q_final,
                                                 const int32_t max_dq)
{
  static double a0, a3, a4, a5; // polynomial coefficients for null init/final vel/acc
  static grabrt::Clock clock;

  // Check if a trajectory was requested
  if (!apply_trajectory_)
    return q_final;

  if (new_trajectory_)
  {
    double dq = q_final - q;
    if (traj_time_ <= 0.0)
      traj_time_ = std::max(1.0, std::abs(dq) / max_dq);
    a0              = q; // this is q_init for a new trajectory
    a3              = 20. / (2 * pow(traj_time_, 3.)) * dq;
    a4              = -30. / (2 * pow(traj_time_, 4.)) * dq;
    a5              = 12. / (2 * pow(traj_time_, 5.)) * dq;
    new_trajectory_ = false;
    clock.Reset();
  }

  double t = clock.Elapsed();
  if (t >= traj_time_)
    return q_final;
  double q_t = a0 + a3 * pow(t, 3.) + a4 * pow(t, 4.) + a5 * pow(t, 5.);
  return static_cast<int32_t>(round(q_t));
}

double ControllerSingleDrive::GetTrajectoryPoint()
{
  static size_t counter = 0;

  if (new_trajectory_)
  {
    counter         = 0;
    new_trajectory_ = false;
  }

  double traj_point = cable_len_traj_[counter++];
  apply_trajectory_ = counter < cable_len_traj_.size();

  return traj_point;
}

void ControllerSingleDrive::Clear()
{
  target_flags_.reset();

  on_target_ = false;

  change_length_target_ = false;
  change_torque_target_ = false;
  delta_length_         = 0.0;

  apply_trajectory_ = false;
}
