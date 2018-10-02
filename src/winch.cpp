#include "winch.h"

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

Winch::Winch(const uint8_t slave_position, WinchParams * const params): servo_(slave_position)
{
  if (params == NULL)
    exit(EXIT_FAILURE);
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

////////////////////////////////////////////////////////////////
/// PulleysSystem Class
////////////////////////////////////////////////////////////////

PulleysSystem::PulleysSystem(PulleyParams * const params)
{
  if (params == NULL)
    exit(EXIT_FAILURE);
  params_ = params;
}

void PulleysSystem::UpdateHomeConfig(const int _home_counts, const double _home_angle)
{
  home_counts_ = _home_counts;
  home_angle_ = _home_angle;
}

void PulleysSystem::UpdateConfig(const int counts)
{
  angle_ = home_angle_ + CountsToPulleyAngleRad(counts - home_counts_);
}
