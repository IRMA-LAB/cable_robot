#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>
#include <stdint.h>

/**
 * @brief The PulleyParams struct
 */
struct PulleyParams
{
  static const double kPulleyResolution;
  static const double kPulleyAngleFactorRad;
  static const double kPulleyAngleFactorDeg;
};

/**
 * @brief The WinchParams struct
 */
struct WinchParams
{
  static constexpr double kDrumPitch = 0.007;  // [m]
  static constexpr double kDrumDiameter = 0.1; // [m]
  static constexpr double kGearRatio = 5.0;
  static const double kShaftResolution;
  static const double kCountsToLengthFactor;
};

/**
 * @brief The ActuatorParams struct
 */
struct ActuatorParams
{
  WinchParams winch;
  PulleyParams pulley;
};

/**
 * @brief The MotorStatus struct
 */
struct MotorStatus {
  int8_t op_mode;
  int32_t motor_position;
  int32_t motor_speed;
  int16_t motor_torque;
};

/**
 * @brief The WinchStatus struct
 */
struct WinchStatus: MotorStatus {
  double cable_length;
  int aux_position;
};

/**
 * @brief The ActuatorStatus struct
 */
struct ActuatorStatus: WinchStatus {
  uint8_t id;
  double pulley_angle;
};

#endif // CABLE_ROBOT_TYPES_H
