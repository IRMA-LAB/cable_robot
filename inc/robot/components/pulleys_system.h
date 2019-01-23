#ifndef CABLE_ROBOT_PULLEYS_SYSTEM_H
#define CABLE_ROBOT_PULLEYS_SYSTEM_H

#include <stdlib.h>

#include "libcdpr/inc/types.h"

#include "utils/types.h"

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
  PulleysSystem(const id_t id, const grabcdpr::PulleyParams& params);

  /**
   * @brief GetAngleRad
   * @return
   */
  double GetAngleRad() const { return angle_; }
  /**
   * @brief GetAngleRad
   * @param counts
   * @return
   */
  double GetAngleRad(const int counts);
  /**
   * @brief GetAngleRad
   * @return
   */
  double GetAngleDeg() const { return angle_ * 180.0 / M_PI; }
  /**
   * @brief GetAngleRad
   * @param counts
   * @return
   */
  double GetAngleDeg(const int counts) { return GetAngleRad(counts) * 180.0 / M_PI; }

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

  /**
   * @brief CountsToPulleyAngleDeg
   * @param counts
   * @return
   */
  inline double CountsToPulleyAngleDeg(const int counts) const;

  /**
   * @brief CountsToPulleyAngleRad
   * @param counts
   * @return
   */
  inline double CountsToPulleyAngleRad(const int counts) const;

private:
  id_t id_;
  grabcdpr::PulleyParams params_;

  int home_counts_ = 0;
  double home_angle_ = 0.0;
  double angle_ = 0.0;
};

#endif // CABLE_ROBOT_PULLEYS_SYSTEM_H
