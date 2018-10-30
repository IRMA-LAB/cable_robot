#include "robot/components/pulleys_system.h"

PulleysSystem::PulleysSystem(const PulleyParams &params)
{
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
