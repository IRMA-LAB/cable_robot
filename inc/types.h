#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>
#include <stdint.h>

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

struct ActuatorParams
{
  WinchParams winch;
  PulleyParams pulley;
};

/**
 * @brief The DriveStatus struct
 */
struct MotorStatus {
  uint8_t motor_id;
  int8_t op_mode;
  double length_target;
  int32_t speed_target;
  int16_t torque_target;
};

#endif // CABLE_ROBOT_TYPES_H
