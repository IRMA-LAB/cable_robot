#include "ctrl/controller_base.h"

ControllerBase::ControllerBase(const uint8_t motor_id)
{
  motors_id_.push_back(motor_id);
  modes_.reserve(1);
}

ControllerBase::ControllerBase(const vect<uint8_t>& motors_id) : motors_id_(motors_id)
{
  modes_.reserve(motors_id.size());
}

ControllerBase::~ControllerBase() {}

////////////////////////////////////////////
/// Public functions
////////////////////////////////////////////

void ControllerBase::SetMotorID(const uint8_t motor_id)
{
  motors_id_.clear();
  modes_.clear();

  motors_id_.push_back(motor_id);
  modes_.reserve(1);
}

void ControllerBase::SetMotorsID(const vect<uint8_t>& motors_id)
{
  motors_id_.clear();
  modes_.clear();

  motors_id_ = motors_id;
  modes_.reserve(motors_id.size());
}

void ControllerBase::SetMode(const ControlMode mode)
{
  if (motors_id_.empty())
    std::cerr << "[ControllerBase] WARNING: no motor ID defined: cannot set motor "
                 "operational mode" << std::endl;
  for (uint8_t i = 0; i < motors_id_.size(); i++)
    modes_[i] = mode;
}

void ControllerBase::SetMode(const uint8_t motor_id, const ControlMode mode)
{
  if (motors_id_.empty())
    std::cerr << "[ControllerBase] WARNING: no motor ID defined: cannot set motor "
                 "operational mode" << std::endl;
  for (uint8_t i = 0; i < motors_id_.size(); i++)
  {
    if (motors_id_[i] == motor_id)
    {
      modes_[i] = mode;
      break;
    }
  }
}

int8_t ControllerBase::GetMode(const uint8_t motor_id) const
{
  for (uint8_t i = 0; i < motors_id_.size(); i++)
    if (motor_id == motors_id_[i])
      return modes_[i];
  return grabec::NONE;
}
