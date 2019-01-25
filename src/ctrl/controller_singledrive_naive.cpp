#include "ctrl/controller_singledrive_naive.h"

ControllerSingleDriveNaive::ControllerSingleDriveNaive(const id_t motor_id,
                                                       const uint32_t period_nsec)
  : ControllerBase(vect<id_t>(1, motor_id)), period_sec_(period_nsec * 0.000000001)
{
  Clear();
  abs_delta_torque_ = period_sec_ * kAbsDeltaTorquePerSec_; // delta per cycle
}

////////////////////////////////////////////////////////////////////////////
//// Public functions
////////////////////////////////////////////////////////////////////////////

void ControllerSingleDriveNaive::SetCableLenTarget(const double target)
{
  Clear();
  length_target_ = target;
  target_flags_.Set(LENGTH);
}

void ControllerSingleDriveNaive::SetMotorPosTarget(const int32_t target)
{
  Clear();
  pos_target_true_ = target;
  target_flags_.Set(POSITION);
}

void ControllerSingleDriveNaive::SetMotorSpeedTarget(const int32_t target)
{
  Clear();
  speed_target_true_ = target;
  target_flags_.Set(SPEED);
}

void ControllerSingleDriveNaive::SetMotorTorqueTarget(const int16_t target)
{
  Clear();
  torque_target_true_ = target;
  target_flags_.Set(TORQUE);
}

void ControllerSingleDriveNaive::CableLenIncrement(const bool active,
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

void ControllerSingleDriveNaive::MotorSpeedIncrement(const Sign sign, const double scale)
{
  static const int32_t kSpeedSpan = kAbsMaxSpeed_ - kAbsMinSpeed_;

  speed_target_true_ = sign * (scale * kSpeedSpan + kAbsMinSpeed_);
}

void ControllerSingleDriveNaive::MotorTorqueIncrement(const Sign sign)
{
  torque_target_true_ += sign * kAbsDeltaTorque_;
}

void ControllerSingleDriveNaive::MotorTorqueIncrement(const bool active,
                                                      const Sign sign /*= Sign::POS*/)
{
  if (active == change_torque_target_)
    return;

  change_torque_target_ = active;
  if (change_torque_target_)
    delta_torque_ = sign * abs_delta_torque_;
}

bool ControllerSingleDriveNaive::CableLenTargetReached(const double current_value)
{
  static const double tol = grabnum::EPSILON; // inserisci una tolleranza vera..
  return grabnum::IsClose(length_target_, current_value, tol);
}

bool ControllerSingleDriveNaive::MotorPosTargetReached(const int32_t current_value)
{
  static const int32_t tol = 1; // inserisci una tolleranza vera..
  return abs(pos_target_true_ - current_value) < tol;
}

bool ControllerSingleDriveNaive::MotorSpeedTargetReached(const int32_t current_value)
{
  static const int32_t tol = 1000; // inserisci una tolleranza vera..
  return abs(speed_target_true_ - current_value) < tol;
}

bool ControllerSingleDriveNaive::MotorTorqueTargetReached(const int16_t current_value)
{
  static const int16_t tol = 5; // per thousand points
  return abs(torque_target_true_ - current_value) < tol;
}

vect<ControlAction> ControllerSingleDriveNaive::CalcCableSetPoint(const grabcdpr::Vars&)
{
  ControlAction res;
  res.ctrl_mode = modes_[0];
  res.motor_id = motors_id_[0];
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
      res.motor_position = pos_target_true_;
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
      }
      res.motor_torque = torque_target_true_;
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

////////////////////////////////////////////////////////////////////////////
//// Private functions
////////////////////////////////////////////////////////////////////////////

void ControllerSingleDriveNaive::Clear()
{
  target_flags_.ClearAll();

  change_length_target_ = false;
  change_torque_target_ = false;
  delta_length_ = 0.0;
}
