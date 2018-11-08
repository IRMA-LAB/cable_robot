#include "ctrl/controller_singledrive_naive.h"

ControllerSingleDriveNaive::ControllerSingleDriveNaive(const uint8_t motor_id)
  : ControllerBase(vect<uint8_t>(1, motor_id))
{
  Clear();
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

void ControllerSingleDriveNaive::SetMotorSpeedTarget(const int32_t target)
{
  Clear();
  speed_target_ = target;
  target_flags_.Set(SPEED);
}

void ControllerSingleDriveNaive::SetMotorTorqueTarget(const int16_t target)
{
  Clear();
  torque_target_ = target;
  target_flags_.Set(TORQUE);
}

void ControllerSingleDriveNaive::CableLenIncrement(const bool active,
                                        const Sign sign /*= Sign::POS*/,
                                        const bool micromove /*= true*/)
{
  change_length_target_ = active;
  if (change_length_target_)
    delta_length_ = micromove ? sign * kDeltaLengthMicro : sign * kDeltaLength;
}

void ControllerSingleDriveNaive::MotorSpeedIncrement(const Sign sign)
{
  speed_target_ += sign * kDeltaSpeed;
}

void ControllerSingleDriveNaive::MotorTorqueIncrement(const Sign sign)
{
  torque_target_ += sign * kDeltaTorque;
}

vect<MotorStatus> ControllerSingleDriveNaive::CalcCableSetPoint(const grabcdpr::Vars&)
{
  MotorStatus res;
  res.op_mode = modes_[0];
  res.motor_id = motors_id_[0];
  switch (res.op_mode)
  {
  case grabec::CYCLIC_POSITION:
    if (target_flags_.CheckBit(LENGTH))
    {
      if (change_length_target_)
        length_target_ += delta_length_;
      res.length_target = length_target_;
    }
    else
      res.op_mode = grabec::NONE;
    break;
  case grabec::CYCLIC_VELOCITY:
    if (target_flags_.CheckBit(SPEED))
      res.speed_target = speed_target_;
    else
      res.op_mode = grabec::NONE;
    break;
  case grabec::CYCLIC_TORQUE:
    if (target_flags_.CheckBit(TORQUE))
      res.torque_target = torque_target_;
    else
      res.op_mode = grabec::NONE;
    break;
  default:
    res.op_mode = grabec::NONE;
    break;
  }
  return vect<MotorStatus>(1, res);
}

////////////////////////////////////////////////////////////////////////////
//// Private functions
////////////////////////////////////////////////////////////////////////////

void ControllerSingleDriveNaive::Clear()
{
  target_flags_.ClearAll();

  change_length_target_ = false;
  delta_length_ = 0.0;
}
