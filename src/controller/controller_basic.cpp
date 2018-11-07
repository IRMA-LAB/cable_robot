#include "controller/controller_basic.h"

ControllerBasic::ControllerBasic(const uint8_t motor_id)
  : ControllerBase(vect<uint8_t>(1, motor_id))
{
  target_flags_.ClearAll();
}

void ControllerBasic::SetCableLenTarget(const double target)
{
  length_target_ = target;
  target_flags_.Set(LENGTH);
}

void ControllerBasic::SetMotorSpeedTarget(const int32_t target)
{
  speed_target_ = target;
  target_flags_.Set(SPEED);
}

void ControllerBasic::SetMotorTorqueTarget(const int16_t target)
{
  torque_target_ = target;
  target_flags_.Set(TORQUE);
}

vect<MotorStatus> ControllerBasic::CalcCableSetPoint(const grabcdpr::Vars&)
{
  MotorStatus res;
  res.op_mode = modes_[0];
  res.motor_id = motors_id_[0];
  switch (res.op_mode)
  {
  case grabec::CYCLIC_POSITION:
    target_flags_.CheckBit(LENGTH) ? res.length_target = length_target_
                                   : res.op_mode = grabec::NULL_OPERATION;
    break;
  case grabec::CYCLIC_VELOCITY:
    target_flags_.CheckBit(SPEED) ? res.speed_target = speed_target_
                                  : res.op_mode = grabec::NULL_OPERATION;
    break;
  case grabec::CYCLIC_TORQUE:
    target_flags_.CheckBit(TORQUE) ? res.torque_target = torque_target_
                                   : res.op_mode = grabec::NULL_OPERATION;
    break;
  default:
    res.op_mode = grabec::NULL_OPERATION;
    break;
  }
  return vect<MotorStatus>(1, res);
}
