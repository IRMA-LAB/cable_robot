/**
 * @file pulleys_system.h
 * @author Simone Comari, Edoardo Idà
 * @date 10 Jan 2020
 * @brief File containing the virtualization of a single pulleys system of the cable
 * robot.
 */

#ifndef CABLE_ROBOT_PULLEYS_SYSTEM_H
#define CABLE_ROBOT_PULLEYS_SYSTEM_H

#include <stdlib.h>

#include "libcdpr/inc/cdpr_types.h"

#include "utils/types.h"

/**
 * @brief The cable robot actuator's pulleys system class.
 *
 * Swivel pulleys system is a component of cable robot actuator.
 * This class includes information about the system technical static parameters and its
 * dynamic configuration, such as angles and encoder counts.
 */
class PulleysSystem
{
 public:
  /**
   * @brief PulleysSystem constructor.
   * @param[in] id Parent actuator ID.
   * @param[in] params Static pulleys system parameters.
   */
  PulleysSystem(const id_t id, const grabcdpr::PulleyParams& params);

  /**
   * @brief Get most recent swivel pulley angle in radians.
   * @return Most recent swivel pulley angle in radians.
   */
  double GetAngleRad() const { return angle_; }
  /**
   * @brief Given encoder counts, return corresponding swivel pulley angle in radians.
   * @param[in] counts Swivel pulley encoder counts.
   * @return Corresponding swivel pulley angle in radians.
   */
  double GetAngleRad(const int counts);
  /**
   * @brief Get most recent swivel pulley angle in degrees.
   * @return Most recent swivel pulley angle in degrees.
   */
  double GetAngleDeg() const { return angle_ * 180.0 / M_PI; }
  /**
   * @brief Given encoder counts, return corresponding swivel pulley angle in degrees.
   * @param[in] counts Swivel pulley encoder counts.
   * @return Corresponding swivel pulley angle in degrees.
   */
  double GetAngleDeg(const int counts) { return GetAngleRad(counts) * 180.0 / M_PI; }

  /**
   * @brief Update pulleys system configuration at home position.
   * @param[in] _home_counts Swivel pulley encoder counts at home position.
   * @param[in] _home_angle swivel pulley angle in radians at home position.
   */
  void UpdateHomeConfig(const int _home_counts, const double _home_angle);
  /**
   * @brief Update swivel pulley angle given corresponding encoder counts.
   * @param[in] counts Swivel pulley encoder counts.
   */
  void UpdateConfig(const int counts);

  /**
   * @brief Convert encoder counts to corresponding swivel pulley angle in degrees.
   * @param[in] counts Swivel pulley encoder counts.
   * @return Corresponding swivel pulley angle in degrees.
   */
  inline double CountsToPulleyAngleDeg(const int counts) const;

  /**
   * @brief Convert encoder counts to corresponding swivel pulley angle in radians.
   * @param[in] counts Swivel pulley encoder counts.
   * @return Corresponding swivel pulley angle in degrees.
   */
  inline double CountsToPulleyAngleRad(const int counts) const;

 private:
  id_t id_;
  grabcdpr::PulleyParams params_;

  int home_counts_   = 0;
  double home_angle_ = 0.0;
  double angle_      = 0.0;
};

#endif // CABLE_ROBOT_PULLEYS_SYSTEM_H
