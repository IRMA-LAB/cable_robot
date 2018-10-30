#ifndef CABLE_ROBOT_PULLEYS_SYSTEM_H
#define CABLE_ROBOT_PULLEYS_SYSTEM_H

#include <stdlib.h>
#include "types.h"

/**
 * @brief The PulleysSystem class
 */
class PulleysSystem
{
public:
  /**
   * @brief PulleysSystem
   * @param params
   */
  PulleysSystem(const PulleyParams& params);

  /**
   * @brief UpdateHomeConfig
   * @param _home_counts
   * @param _home_angle
   */
  void UpdateHomeConfig(const int _home_counts, const double _home_angle);
  /**
   * @brief UpdateConfig
   * @param counts
   */
  void UpdateConfig(const int counts);

private:
  PulleyParams params_;

  int home_counts_ = 0;
  double home_angle_ = 0.0;
  double angle_ = 0.0;

  inline double CountsToPulleyAngleDeg(const int counts) const
  {
    return counts * params_.kPulleyAngleFactorDeg;
  }

  inline double CountsToPulleyAngleRad(const int counts) const
  {
    return counts * params_.kPulleyAngleFactorRad;
  }
};

#endif // CABLE_ROBOT_PULLEYS_SYSTEM_H
