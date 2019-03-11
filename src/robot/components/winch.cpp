/**
 * @file winch.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 11 Mar 2019
 * @brief This file includes definitions of class declared in winch.h.
 */

#include "robot/components/winch.h"

//------------------------------------------------------------------------------------//
//--------- Cable class --------------------------------------------------------------//
//------------------------------------------------------------------------------------//

void Cable::UpdateCableLen(const double delta_length)
{
  length_ = home_length_ + delta_length;
}

double Cable::GetLength(const double delta_length)
{
  UpdateCableLen(delta_length);
  return length_;
}

//------------------------------------------------------------------------------------//
//--------- Winch class --------------------------------------------------------------//
//------------------------------------------------------------------------------------//

Winch::Winch(const id_t id, const uint8_t slave_position,
             const grabcdpr::WinchParams& params)
  : params_(params), servo_(id, slave_position), id_(id)
{}

//--------- Public functions --------------------------------------------------------//

WinchStatus Winch::GetStatus()
{
  WinchStatus status;
  status.id             = id_;
  status.op_mode        = servo_.GetOpMode();
  status.motor_position = servo_.GetPosition();
  status.motor_speed    = servo_.GetVelocity();
  status.motor_torque   = servo_.GetTorque();
  UpdateConfig(status.motor_position);
  status.cable_length = cable_.GetLength();
  status.aux_position = servo_.GetAuxPosition();
  return status;
}

void Winch::SetServoPosByCableLen(const double target_length)
{
  SetServoPos(servo_home_pos_ + LengthToCounts(target_length - cable_.GetHomeLength()));
}

void Winch::SetServoSpeed(const int32_t target_speed)
{
  servo_.ChangeVelocity(target_speed);
}

void Winch::SetServoTorque(const int16_t target_torque)
{
  servo_.ChangeTorque(target_torque);
}

void Winch::SetServoOpMode(const int8_t op_mode) { servo_.ChangeOpMode(op_mode); }

void Winch::UpdateHomeConfig(const double cable_len)
{
  cable_.SetHomeLength(cable_len);
  servo_home_pos_ = servo_.GetPosition();
}

void Winch::UpdateConfig(const int32_t servo_pos)
{
  cable_.UpdateCableLen(CountsToLength(servo_pos - servo_home_pos_));
}

//--------- Private functions --------------------------------------------------------//

double Winch::CountsToLength(const int counts)
{
  return counts * params_.CountsToLengthFactor();
}

int Winch::LengthToCounts(const double length)
{
  return static_cast<int>(length / params_.CountsToLengthFactor());
}
