#include "robot/components/winch.h"

////////////////////////////////////////////////////////////////
/// Cable Class
////////////////////////////////////////////////////////////////

void Cable::UpdateCableLen(const double delta_length)
{
  length_ = home_length_ + delta_length;
}

////////////////////////////////////////////////////////////////
/// Winch Class
////////////////////////////////////////////////////////////////

Winch::Winch(const uint8_t slave_position, const WinchParams &params): servo_(slave_position)
{
  params_ = params;
}

void Winch::SetServoPosByCableLen(const double target_length)
{
  servo_.ChangePosition(servo_start_pos_ + LengthToCounts(target_length));
}

void Winch::SetServoSpeed(const int32_t target_speed)
{
  servo_.ChangeVelocity(target_speed);
}

void Winch::SetServoTorque(const int16_t target_torque)
{
  servo_.ChangeTorque(target_torque);
}

void Winch::UpdateHomeConfig(const double cable_len, const double cable_len_true)
{
  cable_.SetHomeLength(cable_len);
  cable_.SetHomeLengthTrue(cable_len_true);

  servo_home_pos_ = servo_.GetPosition();
}

void Winch::UpdateConfig()
{
  cable_.UpdateCableLen(CountsToLength(servo_.GetPosition()));
}
