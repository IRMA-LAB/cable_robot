#include "robot/components/pulleys_system.h"

PulleysSystem::PulleysSystem(const ID_t id, const grabcdpr::PulleyParams& params)
  : id_(id), params_(params)
{
}

double PulleysSystem::GetAngleRad(const int counts)
{
  UpdateConfig(counts);
  return angle_;
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

double PulleysSystem::CountsToPulleyAngleDeg(const int counts) const
{
  return counts * params_.PulleyAngleFactorDeg();
}

double PulleysSystem::CountsToPulleyAngleRad(const int counts) const
{
  return counts * params_.PulleyAngleFactorRad();
}
