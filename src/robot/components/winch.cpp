/**
 * @file winch.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 02 Mar 2020
 * @brief This file includes definitions of class declared in winch.h.
 */

#include "robot/components/winch.h"

//------------------------------------------------------------------------------------//
//--------- Cable class --------------------------------------------------------------//
//------------------------------------------------------------------------------------//

void Cable::updateCableLen(const double delta_length)
{
  length_ = home_length_ + delta_length;
}

double Cable::getUpdatedLength(const double delta_length)
{
  updateCableLen(delta_length);
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

WinchStatus Winch::getStatus()
{
  WinchStatus status;
  status.id             = id_;
  status.op_mode        = servo_.GetOpMode();
  status.motor_position = servo_.GetPosition();
  status.motor_speed    = servo_.GetVelocity();
  status.motor_torque   = servo_.GetTorque();
  updateConfig(status.motor_position);
  status.cable_length = cable_.getLength();
  status.aux_position = servo_.GetAuxPosition();
  return status;
}

void Winch::setServoPosByCableLen(const double target_length)
{
  setServoPos(servo_home_pos_ + lengthToCounts(target_length - cable_.getHomeLength()));
}

void Winch::setServoSpeed(const int32_t target_speed)
{
  servo_.ChangeVelocity(target_speed);
}

void Winch::setServoTorque(const int16_t target_torque)
{
  servo_.ChangeTorque(target_torque);
}

void Winch::setServoOpMode(const int8_t op_mode) { servo_.ChangeOpMode(op_mode); }

void Winch::updateHomeConfig(const double cable_len)
{
  cable_.setHomeLength(cable_len);
  servo_home_pos_ = servo_.GetPosition();
}

void Winch::updateConfig(const int32_t servo_pos)
{
  cable_.updateCableLen(countsToLength(servo_pos - servo_home_pos_));
}

double Winch::countsToLength(const int counts) const
{
  return counts * params_.transmission_ratio;
}

int Winch::lengthToCounts(const double length) const
{
  return static_cast<int>(length / params_.transmission_ratio);
}
