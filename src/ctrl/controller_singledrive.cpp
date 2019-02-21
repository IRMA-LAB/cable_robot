#include "ctrl/controller_singledrive.h"

ControllerSingleDrive::ControllerSingleDrive(const uint32_t period_nsec)
  : ControllerBase(), period_sec_(period_nsec * 0.000000001),
    pos_ss_err_tol_(kDefaultPosSsErrTol_), torque_ss_err_tol_(kDefaultTorqueSsErrTol_),
    pos_pid_(period_sec_, pos_pid_params_), torque_pid_(period_sec_, torque_pid_params_)
{
  Clear();
  abs_delta_torque_ = period_sec_ * kAbsDeltaTorquePerSec_; // delta per cycle
}

ControllerSingleDrive::ControllerSingleDrive(const id_t motor_id,
                                             const uint32_t period_nsec)
  : ControllerBase(vect<id_t>(1, motor_id)), period_sec_(period_nsec * 0.000000001),
    pos_ss_err_tol_(kDefaultPosSsErrTol_), torque_ss_err_tol_(kDefaultTorqueSsErrTol_),
    pos_pid_(period_sec_, pos_pid_params_), torque_pid_(period_sec_, torque_pid_params_)
{
  Clear();
  abs_delta_torque_ = period_sec_ * kAbsDeltaTorquePerSec_; // delta per cycle
}

//--------- Public functions ---------------------------------------------------------//

void ControllerSingleDrive::SetCableLenTarget(const double target)
{
  Clear();
  length_target_ = target;
  target_flags_.Set(LENGTH);
}

void ControllerSingleDrive::SetMotorPosTarget(const int32_t target)
{
  Clear();
  pos_target_true_ = target;
  target_flags_.Set(POSITION);
}

void ControllerSingleDrive::SetMotorSpeedTarget(const int32_t target)
{
  Clear();
  speed_target_true_ = target;
  target_flags_.Set(SPEED);
}

void ControllerSingleDrive::SetMotorTorqueTarget(const int16_t target)
{
  Clear();
  torque_target_ = static_cast<double>(torque_target_true_);
  target_flags_.Set(TORQUE);
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

bool ControllerSingleDrive::CableLenTargetReached(const double current_value)
{
  static const double tol = grabnum::EPSILON; // inserisci una tolleranza vera..
  return grabnum::IsClose(length_target_, current_value, tol);
}

bool ControllerSingleDrive::MotorPosTargetReached(const int32_t current_value)
{
  static const int32_t tol = 1; // inserisci una tolleranza vera..
  return abs(pos_target_true_ - current_value) < tol;
}

bool ControllerSingleDrive::MotorSpeedTargetReached(const int32_t current_value)
{
  static const int32_t tol = 1000; // inserisci una tolleranza vera..
  return abs(speed_target_true_ - current_value) < tol;
}

vect<ControlAction>
ControllerSingleDrive::CalcCtrlActions(const grabcdpr::Vars&,
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
      if (target_flags_.CheckBit(LENGTH))
      {
        if (change_length_target_)
          length_target_ += delta_length_;
        res.cable_length = length_target_;
      }
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_POSITION:
      if (target_flags_.CheckBit(POSITION))
        res.motor_position = CalcMotorPos(actuators_status);
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_SPEED:
      if (target_flags_.CheckBit(SPEED))
        res.motor_speed = speed_target_true_;
      else
        res.ctrl_mode = NONE;
      break;
    case MOTOR_TORQUE:
      if (target_flags_.CheckBit(TORQUE))
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

  double motor_pos = pos_target_;
  for (const ActuatorStatus& actuator_status : actuators_status)
  {
    if (actuator_status.id != motors_id_[0])
      continue;
    double current_motor_pos = static_cast<double>(actuator_status.motor_position);
    motor_pos                = pos_pid_.Calculate(pos_target_, current_motor_pos);
    printf("%.1f/%.1f --> %d/%d\n", current_motor_pos, pos_target_,
           static_cast<int32_t>(round(motor_pos)), pos_target_true_);
    break;
  }
  on_target_ = (std::abs(pos_pid_.GetError()) + std::abs(pos_pid_.GetPrevError())) <
               (2 * pos_ss_err_tol_);
  //  return static_cast<int32_t>(round(motor_pos));
  return pos_target_true_;
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
    printf("%.1f/%.1f --> %d/%d\n", current_motor_torque, torque_target_,
           static_cast<int16_t>(round(motor_torque)), torque_target_true_);
    break;
  }
  on_target_ = (std::abs(torque_pid_.GetError()) + std::abs(torque_pid_.GetPrevError())) <
               (2 * torque_ss_err_tol_);
  //  return static_cast<int16_t>(round(motor_torque));
  return torque_target_true_;
}

void ControllerSingleDrive::Clear()
{
  target_flags_.ClearAll();

  torque_pid_.Reset();
  on_target_ = false;

  change_length_target_ = false;
  change_torque_target_ = false;
  delta_length_         = 0.0;
}
