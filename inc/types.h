#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>

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

#endif // CABLE_ROBOT_TYPES_H
